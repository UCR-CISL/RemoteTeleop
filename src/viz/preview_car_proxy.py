"""Render local vehicle and proportion galleries through the deployment mesh renderer.

Run with ``python -m src.viz.preview_car_proxy --output-dir artifacts/vehicle-proxies-v3``.
This does not connect to either teleoperation endpoint or load a Gaussian map.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from src.viz.composited_camera import PyTorch3DObjectMeshRenderer
from src.viz.primitive_vehicle import PrimitiveVehicleAsset, SUPPORTED_KINDS
from src.viz.vehicle_proxy import VehicleProxyRenderer


CASES = (
    ("Standard sedan", (4.6, 1.85, 1.5)),
    ("Limo", (9.0, 1.85, 1.5)),
    ("Tall car", (4.6, 1.85, 2.7)),
    ("Wider car", (4.6, 2.5, 1.5)),
    ("Short compact", (2.7, 1.7, 1.5)),
    ("Low sports car", (4.6, 2.0, 1.0)),
    ("Extreme narrow / tall", (3.0, 1.1, 3.0)),
    ("Extreme wide / low", (4.6, 3.5, 0.8)),
)


def _camera(direction, dimensions, image_size):
    # Fit each example individually so shape distortion remains readable.
    target = np.array([0., 0., dimensions[2] / 2.])
    direction = np.asarray(direction, dtype=float)
    direction /= np.linalg.norm(direction)
    forward = -direction
    right = np.cross(forward, [0., 0., 1.])
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    rotation = np.column_stack((right, down, forward))
    width, height = image_size
    focal = height * 1.25
    corners = np.array([[x, y, z] for x in (-.5, .5)
                        for y in (-.5, .5) for z in (-.5, .5)]) * dimensions
    projected = corners @ rotation
    distance = max(np.max(np.abs(projected[:, 0]) * focal / (width * .42) - projected[:, 2]),
                   np.max(np.abs(projected[:, 1]) * focal / (height * .40) - projected[:, 2]))
    pose = np.eye(4)
    pose[:3, :3] = rotation
    pose[:3, 3] = target + direction * distance
    intrinsic = np.array([[focal, 0., width / 2], [0., focal, height / 2], [0., 0., 1.]])
    return pose, intrinsic


def _gallery(cases, output_dir, renderer, *, stem, title, overview_columns=2, views=None):
    import matplotlib.pyplot as plt

    image_size = (800, 500)
    views = views or (("Front three-quarter", (1.3, -1.6, .85)),
             ("Side", (0., -1., .10)), ("Rear three-quarter", (-1.3, -1.6, .85)))
    fig, axes = plt.subplots(len(cases), len(views), figsize=(5 * len(views), 2.7 * len(cases)), facecolor="#f1f4f8", squeeze=False)
    overview_rows = (len(cases) + overview_columns - 1) // overview_columns
    overview, overview_axes = plt.subplots(overview_rows, overview_columns,
        figsize=(6 * overview_columns, 3.7 * overview_rows), facecolor="#f1f4f8", squeeze=False)
    for ax in overview_axes.flat:
        ax.axis("off")
    for row, (name, kind, dimensions) in enumerate(cases):
        model = np.eye(4)
        model[2, 3] = dimensions[2] / 2.
        label = f"{name}  |  {dimensions[0]:g} × {dimensions[1]:g} × {dimensions[2]:g} m"
        for column, (view, direction) in enumerate(views):
            camera, intrinsic = _camera(direction, dimensions, image_size)
            overlay = renderer.render(kind, dimensions, model, camera, intrinsic, image_size)
            alpha = overlay.alpha[..., None]
            rgb = overlay.rgb * alpha + np.array([.945, .957, .973]) * (1. - alpha)
            if not np.any(alpha):
                raise RuntimeError(f"empty preview for {name}: {view}")
            filename = f"{stem}-{row + 1:02d}-{column + 1}.png"
            plt.imsave(output_dir / filename, np.clip(rgb, 0., 1.))
            if column == 0:
                ax = overview_axes.flat[row]
                ax.imshow(rgb)
                ax.set_title(label, fontsize=12)
            ax = axes[row, column]
            ax.imshow(rgb)
            ax.axis("off")
            ax.set_title(label if column == 0 else view, fontsize=11, loc="left")
    fig.suptitle(title, fontsize=23, y=.997)
    fig.text(.5, .979, "Length × width × height • views fitted independently • circular wheels; tiny boxes use fit cap", ha="center", fontsize=11)
    fig.tight_layout(rect=(0., 0., 1., .975), h_pad=1.7)
    fig.savefig(output_dir / f"{stem}-views.png", dpi=130, facecolor=fig.get_facecolor())
    plt.close(fig)
    overview.suptitle(title, fontsize=22)
    overview.tight_layout(rect=(0., 0., 1., .96))
    overview.savefig(output_dir / f"{stem}.png", dpi=130, facecolor=overview.get_facecolor())
    plt.close(overview)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, default=Path("artifacts/vehicle-proxies-v3"))
    parser.add_argument("--device", default="cpu")
    args = parser.parse_args()
    import matplotlib
    matplotlib.use("Agg")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    renderer = VehicleProxyRenderer(PyTorch3DObjectMeshRenderer(device=args.device),
                                   asset_dir=args.output_dir / "meshes")
    vehicles = [(kind.replace("_", " ").title(), kind, PrimitiveVehicleAsset(kind).nominal_dimensions)
                for kind in SUPPORTED_KINDS]
    _gallery(vehicles, args.output_dir, renderer, stem="vehicles", title="Vehicle proxies — class study")
    _gallery([(name, "car", dimensions) for name, dimensions in CASES],
             args.output_dir, renderer, stem="proportions", title="Car proportions — wheels retain their size")
    stress = [
        ("Tiny bbox: wheel fit cap", "car", (1.0, .55, .40)),
        ("Stretched bicycle", "bicycle", (2.8, .65, 1.7)),
        ("Compact truck", "truck", (4.8, 2.0, 2.5)),
        ("Wide loader", "construction_vehicle", (7.0, 3.8, 3.4)),
    ]
    _gallery(stress, args.output_dir, renderer, stem="edge-cases", title="Vehicle proxies — dimension stress tests")
    _gallery([(name, "car", dimensions) for name, dimensions in CASES[:4]],
             args.output_dir, renderer, stem="rear", title="Rear views — C-pillars and tail lights",
             views=(("Rear three-quarter", (-1.3, -1.6, .85)), ("Straight rear", (-1., 0., .10))))
    import matplotlib.pyplot as plt
    detail, axes = plt.subplots(2, 2, figsize=(12, 7.5), facecolor="#f1f4f8")
    for ax, (filename, label) in zip(axes.flat, (
        ("vehicles-01-1.png", "Front — A-pillars and roof rails"),
        ("vehicles-01-2.png", "Side — uninterrupted glazing"),
        ("vehicles-01-3.png", "Rear — C-pillars and roof"),
        ("rear-01-2.png", "Straight rear — tail lights"),
    )):
        ax.imshow(plt.imread(args.output_dir / filename))
        ax.axis("off")
        ax.set_title(label, fontsize=13)
    detail.suptitle("White body, light grey glass — continuous cabin shell", fontsize=19)
    detail.tight_layout(rect=(0, 0, 1, .94))
    detail.savefig(args.output_dir / "cabin-detail.png", dpi=130, facecolor=detail.get_facecolor())
    plt.close(detail)
    print(args.output_dir / "vehicles.png")
    print(args.output_dir / "proportions.png")
    print(args.output_dir / "rear.png")


if __name__ == "__main__":
    main()
