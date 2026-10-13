"""Batch CooperScene detector inference over an mmdet3d info pkl (run in Docker).

Each ``--variant NAME:CONFIG:CHECKPOINT:EXTRINSICS`` builds the checkpoint's
embedded training config and runs its CooperSceneDataset test pipeline unchanged
(file loaders, ImageAug3D, range filter). All variants share one pass over the
data: every LiDAR/image file is read once per sample (the dataset mount is slow
sshfs) and fed to each variant's pipeline. Every prediction with score >=
--min-score is written to ``<output-root>/<NAME>/<split>/predictions.npz`` with a
``manifest.json``. Runs only on recorded data; never touches vehicle services.

Boxes are [x, y, z_gravity_center, l, w, h, yaw] in the sample's LiDAR frame,
the same convention as the pkl ``instances[].bbox_3d``.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import pickle
import sys
import time

import numpy as np

COOPERSCENE_ROOT = Path("/cooperscene")  # thirdparty/CooperScene mount
sys.path.insert(0, "/mmdetection3d")


def _register_modules(calibration: dict | None) -> None:
    from mmcv.transforms import BaseTransform
    from mmdet3d.registry import TRANSFORMS
    from mmdet3d.utils import register_all_modules
    register_all_modules(init_default_scope=True)
    import projects.BEVFusion.bevfusion  # noqa: F401  (source-identical to CooperScene's; loading.py differs only for KITTI)
    # Register CooperSceneDataset without importing CooperScene's cooperative
    # package (which pulls OpenCOOD); the file only depends on mmdet3d.
    path = COOPERSCENE_ROOT / "models/cooperative/datasets/cooperscene_dataset.py"
    spec = importlib.util.spec_from_file_location("cooperscene_dataset", path)
    spec.loader.exec_module(importlib.util.module_from_spec(spec))

    @TRANSFORMS.register_module(force=True)
    class OverrideCameraCalibration(BaseTransform):
        """Replace camera0 lidar2cam/cam2img by per-(take, agent) calibration."""

        def transform(self, results):
            entry = calibration[f"scene{results['scenario']}_agent{results['agent_id']}"]
            cam = results["images"]["camera0"]
            cam["lidar2cam"] = np.asarray(entry["T_lidar_to_cam"], dtype=np.float64).tolist()
            cam["cam2img"] = np.asarray(entry["intrinsic"], dtype=np.float64).tolist()
            cam.pop("lidar2img", None)
            return results


class SharedFileReads:
    """Memoize mmengine ``get`` inside the loaders for one sample, so several
    variant pipelines decode the same bytes but read the slow mount once."""

    def __init__(self) -> None:
        import mmengine.fileio
        import mmdet3d.datasets.transforms.loading as det3d_loading
        import projects.BEVFusion.bevfusion.loading as bev_loading
        self._get, self._cache = mmengine.fileio.get, {}
        for module in (det3d_loading, bev_loading):
            module.get = self._cached_get

    def _cached_get(self, path, backend_args=None):
        if path not in self._cache:
            self._cache[path] = self._get(path, backend_args=backend_args)
        return self._cache[path]

    def clear(self) -> None:
        self._cache.clear()


class MultiVariantDataset:
    """torch-style dataset returning one packed input per variant pipeline."""

    def __init__(self, base, pipelines: list) -> None:
        self.base, self.pipelines, self.reads = base, pipelines, None

    def __len__(self) -> int:
        return len(self.base)

    def __getitem__(self, index: int) -> list:
        if self.reads is None:  # per worker process
            self.reads = SharedFileReads()
        info = self.base.get_data_info(index)
        info.update(box_type_3d=self.base.box_type_3d, box_mode_3d=self.base.box_mode_3d)
        try:
            return [pipeline(copy.deepcopy(info)) for pipeline in self.pipelines]
        finally:
            self.reads.clear()


class Variant:
    """One (checkpoint, extrinsics) detector with its pipeline and outputs."""

    def __init__(self, spec: str, models: dict) -> None:
        from mmengine.config import Config
        from mmdet3d.registry import MODELS
        self.name, self.config, self.checkpoint, self.extrinsics = spec.split(":")
        if self.extrinsics not in ("baseline", "benchmark"):
            raise SystemExit(f"bad extrinsics in {spec}")
        # The checkpoint's custom_imports name CooperScene's package; the
        # installed projects.BEVFusion is registered instead (see above).
        self.cfg = Config.fromfile(self.config, import_custom_modules=False)
        pipeline = list(self.cfg.test_dataloader.dataset.pipeline)
        pipeline[-1] = dict(pipeline[-1], keys=[k for k in pipeline[-1]["keys"] if not k.startswith("gt_")])
        self.has_camera = any(t["type"] == "BEVLoadMultiViewImageFromFiles" for t in pipeline)
        if self.extrinsics == "benchmark":
            if not self.has_camera:
                raise SystemExit("benchmark extrinsics require a camera model")
            pipeline.insert(0, dict(type="OverrideCameraCalibration"))
        self.pipeline = pipeline
        key = (self.config, self.checkpoint)
        if key not in models:
            cfg = self.cfg.model.copy()
            cfg.pop("init_cfg", None)
            if "img_backbone" in cfg:
                cfg.img_backbone.init_cfg = None  # weights come from the checkpoint
            model = MODELS.build(cfg)
            info = self._load_checkpoint(model, self.checkpoint)
            models[key] = (model.to("cuda:0").eval(), info)
        self.model, self.checkpoint_info = models[key]
        self.rows = dict(keys=[], boxes=[], scores=[], labels=[], counts=[], forward_ms=[])

    @staticmethod
    def _load_checkpoint(model, path: str) -> dict:
        """Strict load; spconv2 (O,D,H,W,I) sparse weights -> mmcv (D,H,W,I,O) in memory."""
        import torch
        ckpt = torch.load(path, map_location="cpu")
        state = ckpt.get("state_dict", ckpt)
        target = model.state_dict()
        converted = 0
        for name, value in list(state.items()):
            if value.ndim == 5 and name in target and value.shape != target[name].shape:
                state[name] = value.permute(1, 2, 3, 4, 0).contiguous()
                converted += 1
        model.load_state_dict(state, strict=True)
        meta = ckpt.get("meta", {})
        return dict(path=path, sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest(), tensors=len(state),
                    sparse_layout_converted=converted, epoch=meta.get("epoch"),
                    classes=meta.get("dataset_meta", {}).get("classes"))

    def infer(self, sample, key, min_score: float, fp16: bool) -> None:
        import torch
        torch.cuda.synchronize()
        started = time.perf_counter()
        with torch.inference_mode(), torch.autocast("cuda", dtype=torch.float16, enabled=fp16):
            pred = self.model.test_step(sample)[0].pred_instances_3d
        torch.cuda.synchronize()
        self.rows["forward_ms"].append((time.perf_counter() - started) * 1000)
        keep = pred.scores_3d >= min_score
        b = pred.bboxes_3d[keep]
        out = torch.cat([b.gravity_center, b.tensor[:, 3:7]], dim=1).float().cpu().numpy()
        self.rows["boxes"].append(out)
        self.rows["scores"].append(pred.scores_3d[keep].float().cpu().numpy())
        self.rows["labels"].append(pred.labels_3d[keep].cpu().numpy().astype(np.int16))
        self.rows["counts"].append(len(out))
        self.rows["keys"].append(key)

    def write(self, out: Path, extra: dict, gt_points: list, wall_s: float, manifest: dict) -> None:
        out.mkdir(parents=True, exist_ok=True)
        r = self.rows
        fwd = np.asarray(r["forward_ms"])
        np.savez_compressed(
            out / "predictions.npz",
            scenario=np.array([k[0] for k in r["keys"]]), agent_id=np.array([k[1] for k in r["keys"]]),
            timestamp=np.array([k[2] for k in r["keys"]]), sample_idx=np.array([k[3] for k in r["keys"]]),
            counts=np.asarray(r["counts"], np.int32),
            boxes=np.concatenate(r["boxes"]).astype(np.float32), scores=np.concatenate(r["scores"]),
            labels=np.concatenate(r["labels"]), forward_ms=fwd.astype(np.float32),
            gt_num_points=np.concatenate(gt_points), gt_counts=np.asarray([len(g) for g in gt_points], np.int32),
            **extra)
        warm = min(5, len(fwd) - 1)
        steady = fwd[warm:]
        manifest = dict(
            manifest, variant=self.name, config=self.config, extrinsics=self.extrinsics,
            config_sha256=hashlib.sha256(Path(self.config).read_bytes()).hexdigest(),
            checkpoint=self.checkpoint_info, pipeline=self.pipeline, samples=len(fwd), boxes=int(sum(r["counts"])),
            box_format="[x, y, z_gravity_center, l(x_size), w(y_size), h, yaw] LiDAR frame",
            shared_pass_wall_time_s=wall_s,
            forward_ms=dict(mean=float(steady.mean()), median=float(np.median(steady)),
                            p95=float(np.percentile(steady, 95)), max=float(steady.max()),
                            first=float(fwd[0]), excluded_warmup=warm))
        (out / "manifest.json").write_text(json.dumps(manifest, indent=2, default=str) + "\n")
        print(self.name, json.dumps({k: manifest[k] for k in ("samples", "boxes", "forward_ms")}), flush=True)


class BatchDetector:
    """Owns variants, the shared data pass and the timed inference loop."""

    def __init__(self, args: argparse.Namespace) -> None:
        from mmdet3d.registry import DATASETS
        self.args = args
        benchmark = any(spec.split(":")[-1] == "benchmark" for spec in args.variant)
        _register_modules(json.loads(Path(args.benchmark_calibration).read_text()) if benchmark else None)
        models: dict = {}
        self.variants = [Variant(spec, models) for spec in args.variant]
        # Base dataset parses infos (paths/calibration) once; prefer a camera
        # config so image entries are parsed for the camera variants.
        base = next((v for v in self.variants if v.has_camera), self.variants[0])
        ds_cfg = base.cfg.test_dataloader.dataset.copy()
        ds_cfg.update(ann_file=args.ann_file, pipeline=[], test_mode=True, data_root=args.image_root)
        self.dataset = DATASETS.build(ds_cfg)
        from mmengine.dataset import Compose
        self.data = MultiVariantDataset(self.dataset, [Compose(v.pipeline) for v in self.variants])

    def run(self) -> None:
        import torch
        from torch.utils.data import DataLoader, Subset
        from mmengine.dataset import pseudo_collate
        from mmdet3d.structures import LiDARInstance3DBoxes
        args = self.args
        infos = pickle.load(open(args.ann_file, "rb"))["data_list"]
        assert len(infos) == len(self.dataset), "dataset must not filter samples in test mode"
        indices = list(range(len(infos)))[args.start:]
        if args.limit:
            indices = indices[:args.limit]
        loader = DataLoader(Subset(self.data, indices), batch_size=1, num_workers=args.num_workers,
                            collate_fn=lambda batch: batch[0], prefetch_factor=4 if args.num_workers else None)
        gt_points, fetch_ms = [], []
        wall_start = fetch_start = time.perf_counter()
        for position, (index, samples) in enumerate(zip(indices, loader)):
            fetch_ms.append((time.perf_counter() - fetch_start) * 1000)
            info = infos[index]
            key = (str(info["scenario"]), str(info["agent_id"]), float(info["timestamp"]), int(info["sample_idx"]))
            for variant, sample in zip(self.variants, samples):
                variant.infer(pseudo_collate([sample]), key, args.min_score, args.fp16)
            # Evaluation-only GT observability: LiDAR points (after the model's
            # range filter) inside each pkl GT box. Never fed to any model.
            gt = np.asarray([inst["bbox_3d"] for inst in info.get("instances", [])], dtype=np.float32).reshape(-1, 7)
            if len(gt):
                pts = samples[0]["inputs"]["points"][:, :3].cuda()
                gt_boxes = LiDARInstance3DBoxes(torch.from_numpy(gt), origin=(0.5, 0.5, 0.5)).to("cuda")
                gt_points.append(gt_boxes.points_in_boxes_all(pts).sum(0).cpu().numpy().astype(np.int32))
            else:
                gt_points.append(np.zeros(0, np.int32))
            if position % 200 == 0:
                fwd = " ".join(f"{v.name}={v.rows['forward_ms'][-1]:.0f}ms" for v in self.variants)
                print(f"[{position}/{len(indices)}] {time.perf_counter() - wall_start:.0f}s fetch {fetch_ms[-1]:.0f}ms {fwd}", flush=True)
            fetch_start = time.perf_counter()
        wall_s = time.perf_counter() - wall_start
        split = Path(args.ann_file).stem.replace("cooperscene_infos_", "")
        manifest = dict(argv=sys.argv, args=vars(args), split=split,
                        ann_file_sha256=hashlib.sha256(Path(args.ann_file).read_bytes()).hexdigest(),
                        gpu=torch.cuda.get_device_name(0), variants_in_pass=[v.name for v in self.variants],
                        fetch_wait_ms_mean=float(np.mean(fetch_ms)))
        for variant in self.variants:
            variant.write(Path(args.output_root) / variant.name / split, dict(fetch_ms=np.asarray(fetch_ms, np.float32)),
                          gt_points, wall_s, manifest)
        print(f"shared pass wall time {wall_s:.1f}s for {len(indices)} samples", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", action="append", required=True,
                        help="NAME:CONFIG:CHECKPOINT:{baseline,benchmark}; CONFIG is the checkpoint-embedded training config")
    parser.add_argument("--ann-file", required=True)
    parser.add_argument("--image-root", default="/workspace/data/Cooperscene/release/250928_opv2v/",
                        help="dataset data_root (image prefix); the training root by default")
    parser.add_argument("--benchmark-calibration", default="/workspace/data/Cooperscene/benchmark_cooperbevfusion/250928/calibration_results/calibration_results_all.json")
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--min-score", type=float, default=0.05)
    parser.add_argument("--num-workers", type=int, default=8)
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--fp16", action="store_true", help="autocast forward (timing experiment)")
    BatchDetector(parser.parse_args()).run()


if __name__ == "__main__":
    main()
