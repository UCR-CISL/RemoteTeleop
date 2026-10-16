"""Offline LiDAR free-space carving of a splat map's vehicle-proxy occluder layer.

Low-opacity splat floaters hang 0.3-2 m above driven lanes and, together, hide the vehicle proxies
that the compositor depth-tests against ``OpacityThresholdDepth``.  Logged LiDAR scans, registered
into the splat frame, show that space is empty: every ray passes through it on the way to a return.

For every scan, rays are cast from the sensor to each return.  Voxels (``--voxel-m``) holding a splat
centre count a *free* traversal when a ray passes them at least ``--free-margin-m`` before its return,
and a *hit* when a return lands in them or a horizontally neighbouring voxel (not the voxels above:
road returns would otherwise "hit" floaters 0.3-0.5 m over the lane).  A splat is a floater when its
voxel was traversed at least ``--min-free`` times and hits are at most ``--max-hit-fraction`` of
``free + hit``; never-traversed space and space near returns stay occluders.  Returns on *moving*
vehicles (GT boxes whose centre moves more than ``--moving-min-m`` over +-0.5 s) still end their rays
but are not hits: traffic passes through the lanes where the floaters hang.  Parked vehicles keep
their hits, so vehicles baked into the map keep occluding.

The result is a sidecar ``<map>.occluder_mask.npz`` (``occluder_mask``: per-splat bool, True =
occluder; ``ply_sha256``; ``num_splats``; ``params``) consumed by
``OpacityThresholdDepth.load_occluder_mask``.  Only the occluder pass changes; the color render keeps
every splat.  Like map training this is map preparation, so it uses every logged scan.
"""
from __future__ import annotations

import argparse
import json
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from pathlib import Path

import numpy as np

MAIN_REPO = Path("/home/jyue86/Documents/CISL-Projects/RemoteTeleop")
DEFAULT_MAP = MAIN_REPO / "artifacts/reference-map-20261011/riverside_iowa.ply"
DEFAULT_ALIGNMENT = MAIN_REPO / "artifacts/worktree-archive-20261011/alignment_artifacts/recipe-frame-search-full-20261004"
DEFAULT_DATA = Path("/mnt/bcc-data/data/CooperScene")
MASK_VERSION = 1
_BITS = 21
_OFFSET = 1 << (_BITS - 1)


class FreeSpaceCarver:
    """Accumulate per-group LiDAR free/hit counts at the voxels holding splat centres (GPU)."""

    def __init__(self, means: np.ndarray, *, voxel_m: float = 0.2, free_margin_m: float = 0.4,
                 min_range_m: float = 1.5, max_range_m: float = 40.0, groups: int = 1, device: str = "cuda") -> None:
        import torch

        self.voxel_m, self.free_margin_m = float(voxel_m), float(free_margin_m)
        self.min_range_m, self.max_range_m = float(min_range_m), float(max_range_m)
        self.device = device
        keys = self._encode(torch.as_tensor(np.asarray(means), dtype=torch.float32, device=device))
        self.keys, self.splat_voxel = torch.unique(keys, return_inverse=True)
        self.free = torch.zeros((groups, len(self.keys)), dtype=torch.int32, device=device)
        self.hit = torch.zeros_like(self.free)
        offsets = torch.stack(torch.meshgrid(*(torch.arange(-1, 2),) * 2, indexing="ij"), -1).reshape(-1, 2)
        self._neighbours = ((offsets[:, 0] << 2 * _BITS) + (offsets[:, 1] << _BITS)).to(device)  # 3x3x1

    def _encode(self, points):
        import torch

        cell = torch.floor(points / self.voxel_m).long() + _OFFSET
        cell = cell.clamp(0, (1 << _BITS) - 1)
        return (cell[..., 0] << 2 * _BITS) | (cell[..., 1] << _BITS) | cell[..., 2]

    def _count(self, counts, group: int, queries) -> None:
        import torch

        pos = torch.searchsorted(self.keys, queries).clamp(max=len(self.keys) - 1)
        found = self.keys[pos] == queries
        counts[group].index_add_(0, pos[found], torch.ones_like(pos[found], dtype=torch.int32))

    def add_scan(self, points_world: np.ndarray, origin: np.ndarray, group: int = 0, *,
                 hit_mask: np.ndarray | None = None, max_rays: int | None = None, chunk: int = 4096,
                 rng: np.random.Generator | None = None) -> None:
        """Count one scan; ``hit_mask`` False marks returns that end rays without counting as hits."""

        import torch

        pts = torch.as_tensor(np.asarray(points_world, dtype=np.float32), device=self.device)
        o = torch.as_tensor(np.asarray(origin, dtype=np.float32), device=self.device)
        ray = pts - o
        rng_m = ray.norm(dim=1)
        ok = rng_m > self.min_range_m
        hits = ok if hit_mask is None else ok & torch.as_tensor(np.asarray(hit_mask, bool), device=self.device)
        self._count(self.hit, group, (self._encode(pts[hits])[:, None] + self._neighbours[None]).reshape(-1))
        ray, rng_m = ray[ok], rng_m[ok]
        if max_rays is not None and len(ray) > max_rays:
            pick = (rng or np.random.default_rng()).choice(len(ray), max_rays, replace=False)
            pick = torch.as_tensor(pick, device=self.device)
            ray, rng_m = ray[pick], rng_m[pick]
        direction = ray / rng_m[:, None]
        step = self.voxel_m / 2
        t = torch.arange(self.min_range_m, self.max_range_m, step, device=self.device)
        free_until = (rng_m - self.free_margin_m).clamp(max=self.max_range_m)
        for start in range(0, len(direction), chunk):
            d, until = direction[start:start + chunk], free_until[start:start + chunk]
            keys = self._encode(o + d[:, None, :] * t[None, :, None])
            first = torch.ones_like(keys, dtype=torch.bool)
            first[:, 1:] = keys[:, 1:] != keys[:, :-1]  # count each traversed voxel once per ray
            self._count(self.free, group, keys[first & (t[None] < until[:, None])])

    def splat_counts(self, groups=None) -> tuple[np.ndarray, np.ndarray]:
        """Per-splat (free, hit) summed over ``groups`` (default all)."""

        free = self.free if groups is None else self.free[list(groups)]
        hit = self.hit if groups is None else self.hit[list(groups)]
        return (free.sum(0)[self.splat_voxel].cpu().numpy(), hit.sum(0)[self.splat_voxel].cpu().numpy())

    @staticmethod
    def occluder_mask(free: np.ndarray, hit: np.ndarray, *, min_free: int = 8,
                      max_hit_fraction: float = 0.05) -> np.ndarray:
        """True keeps a splat as an occluder; False marks a carved floater."""

        free, hit = np.asarray(free, np.int64), np.asarray(hit, np.int64)
        return ~((free >= min_free) & (hit <= max_hit_fraction * (free + hit)))


@dataclass(frozen=True)
class AlignedScan:
    """One logged CooperScene scan with its accepted splat-frame registration."""

    pcd: Path
    take: int
    agent: int
    index: int
    map_T_lidar: np.ndarray

    @classmethod
    def from_alignment(cls, alignment: Path, data_root: Path, *, stride: int = 1) -> list[AlignedScan]:
        scans = []
        for path in sorted(alignment.glob("take*/take*-vehicle*/frame-*.json")):
            if int(path.stem.split("-")[1]) % stride:
                continue
            item = json.loads(path.read_text())
            method = item["methods"]["xyz"]
            if not method.get("accepted"):
                continue
            frame = item["frame"]
            split = Path(frame["archive"]).parent.name
            pcd = data_root / split / str(frame["take"]) / str(frame["agent"]) / f"{frame['frame_id']}.pcd"
            scans.append(cls(pcd, int(frame["take"]), int(frame["agent"]), int(frame["frame_index"]),
                             np.asarray(method["error"]["estimated_map_T_lidar"], dtype=np.float64).reshape(4, 4)))
        return scans

    def world_points(self) -> np.ndarray:
        with self.pcd.open("rb") as stream:
            for line in stream:
                if line.startswith(b"DATA"):
                    break
            values = np.fromstring(stream.read(), dtype=np.float32, sep=" ").reshape(-1, 4)[:, :3]
        values = values[np.isfinite(values).all(axis=1)]
        return (values @ self.map_T_lidar[:3, :3].T + self.map_T_lidar[:3, 3]).astype(np.float32)

    def static_mask(self, points_world: np.ndarray, *, span: int = 5, min_motion_m: float = 1.0,
                    inflate_m: float = 0.3) -> np.ndarray:
        """False for returns inside a GT vehicle box (inflated) whose centre moves more than
        ``min_motion_m`` between native frames ``index -+ span``."""

        sequence = _sequence(self.pcd.parent)
        frame = sequence[self.index]
        assert frame.frame_id == self.pcd.stem, (frame.frame_id, self.pcd)
        near = [sequence[i] for i in (max(self.index - span, 0), min(self.index + span, len(sequence) - 1))]
        centres = [{box.track_id: box.map_T_object[:3, 3] for box in other.vehicle_boxes} for other in near]
        native_to_splat = self.map_T_lidar @ np.linalg.inv(frame.map_T_lidar)
        static = np.ones(len(points_world), bool)
        for box in frame.vehicle_boxes:
            track = [c[box.track_id] for c in centres if box.track_id in c] + [box.map_T_object[:3, 3]]
            if len(track) < 2 or np.linalg.norm(track[0] - track[1]) <= min_motion_m:  # -+span, else one vs now
                continue
            splat_T_object = native_to_splat @ box.map_T_object
            local = (points_world - splat_T_object[:3, 3]) @ splat_T_object[:3, :3]
            static &= ~np.all(np.abs(local) <= np.asarray(box.dimensions_lwh) / 2 + inflate_m, axis=1)
        return static


_SEQUENCES: dict = {}


def _sequence(path: Path):
    """Metadata-only native sequence for a ``<split>/<take>/<agent>`` directory (cached per process)."""

    if path not in _SEQUENCES:
        from src.localization.cooperscene import CooperSceneSequenceDataset

        dataset = CooperSceneSequenceDataset.metadata_only(path.parents[2], cache_frames=False)
        _SEQUENCES[path] = dataset.sequence(path.parents[1].name, path.parent.name, path.name)
    return _SEQUENCES[path]


def _load(scan: AlignedScan) -> tuple[np.ndarray, np.ndarray]:
    points = scan.world_points()
    return points, scan.static_mask(points)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--alignment", type=Path, default=DEFAULT_ALIGNMENT,
                        help="per-frame splat-frame registrations (accepted 'xyz' method poses)")
    parser.add_argument("--data-root", type=Path, default=DEFAULT_DATA)
    parser.add_argument("--output", type=Path, help="default: <map without .ply>.occluder_mask.npz")
    parser.add_argument("--counts-output", type=Path, help="optionally save per-take free/hit counts (evaluation)")
    parser.add_argument("--stride", type=int, default=1, help="use every N-th aligned frame")
    parser.add_argument("--max-rays", type=int, default=40_000, help="rays sampled per scan for free-space counts")
    parser.add_argument("--voxel-m", type=float, default=0.2)
    parser.add_argument("--free-margin-m", type=float, default=0.4)
    parser.add_argument("--max-range-m", type=float, default=40.0)
    parser.add_argument("--min-free", type=int, default=8)
    parser.add_argument("--max-hit-fraction", type=float, default=0.05)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--device", default="cuda")
    args = parser.parse_args()
    from src.localization.gaussian_map import load_gaussian_map
    from src.viz.composited_camera import OpacityThresholdDepth

    start = time.time()
    gaussian_map = load_gaussian_map(args.map)
    scans = AlignedScan.from_alignment(args.alignment, args.data_root, stride=args.stride)
    takes = sorted({scan.take for scan in scans})
    carver = FreeSpaceCarver(gaussian_map.means, voxel_m=args.voxel_m, free_margin_m=args.free_margin_m,
                             max_range_m=args.max_range_m, groups=len(takes), device=args.device)
    print(f"{len(scans)} aligned scans over takes {takes}; {len(carver.keys)} splat voxels", flush=True)
    rng = np.random.default_rng(0)
    with ProcessPoolExecutor(args.workers) as pool:
        for i, (scan, (points, static)) in enumerate(zip(scans, pool.map(_load, scans, chunksize=4))):
            carver.add_scan(points, scan.map_T_lidar[:3, 3], takes.index(scan.take), hit_mask=static,
                            max_rays=args.max_rays, rng=rng)
            if i % 500 == 0:
                print(f"  {i}/{len(scans)} scans  {time.time() - start:.0f} s", flush=True)
    free, hit = carver.splat_counts()
    mask = FreeSpaceCarver.occluder_mask(free, hit, min_free=args.min_free, max_hit_fraction=args.max_hit_fraction)
    params = {k: (str(v) if isinstance(v, Path) else v) for k, v in vars(args).items()}
    params.update(scans=len(scans), takes=takes, runtime_s=round(time.time() - start, 1))
    output = args.output or args.map.with_suffix(".occluder_mask.npz")
    np.savez_compressed(output, occluder_mask=mask, version=MASK_VERSION, ply_sha256=OpacityThresholdDepth.splat_map_sha256(args.map),
                        num_splats=len(mask), params=json.dumps(params))
    if args.counts_output is not None:
        np.savez_compressed(args.counts_output, takes=np.asarray(takes), splat_voxel=carver.splat_voxel.cpu().numpy(),
                            free=carver.free.cpu().numpy(), hit=carver.hit.cpu().numpy())
    print(f"wrote {output}: {int((~mask).sum())}/{len(mask)} splats ({(~mask).mean():.2%}) carved out of the "
          f"occluder layer in {time.time() - start:.0f} s", flush=True)


if __name__ == "__main__":
    main()
