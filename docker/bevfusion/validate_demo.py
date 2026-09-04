#!/usr/bin/env python3
"""Validate a BEVFusion checkpoint against MMDetection3D's nuScenes demo."""

from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
import json
from pathlib import Path
import pickle

import matplotlib.pyplot as plt
import numpy as np
import torch


CLASSES = (
    "car", "truck", "construction_vehicle", "bus", "trailer", "barrier",
    "motorcycle", "bicycle", "pedestrian", "traffic_cone",
)


def _match(iou: np.ndarray, gt_labels: np.ndarray, pred_labels: np.ndarray, threshold: float) -> int:
    candidates = [
        (float(iou[gt, pred]), gt, pred)
        for gt in range(len(gt_labels))
        for pred in range(len(pred_labels))
        if gt_labels[gt] == pred_labels[pred] and iou[gt, pred] >= threshold
    ]
    used_gt: set[int] = set()
    used_pred: set[int] = set()
    matches = 0
    for _, gt, pred in sorted(candidates, reverse=True):
        if gt not in used_gt and pred not in used_pred:
            used_gt.add(gt)
            used_pred.add(pred)
            matches += 1
    return matches


def _corners(box: np.ndarray) -> np.ndarray:
    x, y, _, length, width, _, yaw = box[:7]
    local = np.asarray(
        ((length / 2, width / 2), (length / 2, -width / 2),
         (-length / 2, -width / 2), (-length / 2, width / 2),
         (length / 2, width / 2)),
        dtype=np.float64,
    )
    rotation = np.asarray(
        ((np.cos(yaw), -np.sin(yaw)), (np.sin(yaw), np.cos(yaw))),
        dtype=np.float64,
    )
    return local @ rotation.T + (x, y)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True)
    parser.add_argument("--demo-root", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--score-threshold", type=float, default=0.2)
    args = parser.parse_args()

    from mmcv.ops import box_iou_rotated
    from mmengine.config import Config
    from mmengine.dataset import pseudo_collate
    from mmdet3d.apis import init_model
    from mmdet3d.registry import DATASETS

    info_path = next(args.demo_root.glob("*.pkl"))
    with info_path.open("rb") as stream:
        info = pickle.load(stream)

    config = Config.fromfile(str(args.config))
    model = init_model(config, str(args.checkpoint), device=args.device)
    dataset_config = deepcopy(config.test_dataloader.dataset)
    dataset_config.data_root = str(args.demo_root) + "/"
    dataset_config.ann_file = info_path.name
    dataset_config.data_prefix = {key: "" for key in dataset_config.data_prefix}
    dataset = DATASETS.build(dataset_config)

    with torch.inference_mode():
        result = model.test_step(pseudo_collate([dataset[0]]))[0]

    instances = result.pred_instances_3d
    scores = instances.scores_3d.detach().cpu().numpy()
    keep = scores >= args.score_threshold
    pred_boxes = instances.bboxes_3d.tensor.detach().cpu().numpy()[keep, :7]
    pred_labels = instances.labels_3d.detach().cpu().numpy()[keep]

    source_labels = {
        int(label): name for name, label in info["metainfo"]["categories"].items()
    }
    model_labels = {name: index for index, name in enumerate(CLASSES)}
    raw_instances = [
        item for item in info["data_list"][0]["instances"]
        if int(item["bbox_label_3d"]) in source_labels
        and source_labels[int(item["bbox_label_3d"])] in model_labels
    ]
    gt_boxes = np.asarray([item["bbox_3d"] for item in raw_instances], dtype=np.float32)
    gt_labels = np.asarray(
        [model_labels[source_labels[int(item["bbox_label_3d"])]] for item in raw_instances],
        dtype=np.int64,
    )
    point_cloud_range = np.asarray(config.point_cloud_range, dtype=np.float32)
    in_range = (
        (gt_boxes[:, 0] >= point_cloud_range[0])
        & (gt_boxes[:, 0] <= point_cloud_range[3])
        & (gt_boxes[:, 1] >= point_cloud_range[1])
        & (gt_boxes[:, 1] <= point_cloud_range[4])
    )
    gt_boxes, gt_labels = gt_boxes[in_range], gt_labels[in_range]

    gt_bev = torch.as_tensor(gt_boxes[:, [0, 1, 3, 4, 6]], device=args.device)
    pred_bev = torch.as_tensor(pred_boxes[:, [0, 1, 3, 4, 6]], device=args.device)
    iou = box_iou_rotated(gt_bev, pred_bev).detach().cpu().numpy()
    same_class = gt_labels[:, None] == pred_labels[None, :]
    best = np.where(same_class, iou, 0.0).max(axis=1) if len(pred_boxes) else np.zeros(len(gt_boxes))

    metrics = {}
    for threshold in (0.3, 0.5, 0.7):
        true_positive = _match(iou, gt_labels, pred_labels, threshold)
        metrics[str(threshold)] = {
            "tp": true_positive,
            "fp": len(pred_boxes) - true_positive,
            "fn": len(gt_boxes) - true_positive,
            "precision": true_positive / len(pred_boxes) if len(pred_boxes) else 0.0,
            "recall": true_positive / len(gt_boxes) if len(gt_boxes) else 0.0,
        }

    payload = {
        "score_threshold": args.score_threshold,
        "gt_in_model_xy_range": len(gt_boxes),
        "predictions": len(pred_boxes),
        "gt_classes": dict(sorted(Counter(CLASSES[int(label)] for label in gt_labels).items())),
        "prediction_classes": dict(sorted(Counter(CLASSES[int(label)] for label in pred_labels).items())),
        "mean_best_iou_per_gt": float(best.mean()),
        "median_best_iou_per_gt": float(np.median(best)),
        "metrics": metrics,
    }
    args.output_root.mkdir(parents=True, exist_ok=True)
    metrics_path = args.output_root / "bevfusion_demo_bbox_metrics.json"
    metrics_path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")

    figure, axis = plt.subplots(figsize=(10, 10))
    for box in gt_boxes:
        corners = _corners(box)
        axis.plot(corners[:, 0], corners[:, 1], color="tab:green", linewidth=1.2)
    for box in pred_boxes:
        corners = _corners(box)
        axis.plot(corners[:, 0], corners[:, 1], color="tab:red", linewidth=1.0)
    axis.plot([], [], color="tab:green", label="ground truth")
    axis.plot([], [], color="tab:red", label=f"prediction >= {args.score_threshold:g}")
    axis.set(xlim=(-54, 54), ylim=(-54, 54), xlabel="LiDAR x (m)", ylabel="LiDAR y (m)")
    axis.set_aspect("equal")
    axis.grid(alpha=0.2)
    axis.legend()
    figure.tight_layout()
    figure.savefig(args.output_root / "bevfusion_demo_bbox_overlay.png", dpi=160)
    print(json.dumps(payload, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
