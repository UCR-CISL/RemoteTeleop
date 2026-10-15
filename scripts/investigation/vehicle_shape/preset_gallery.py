"""Render every universal vehicle type mean through VehicleProxyRenderer (PyTorch3D) into one gallery PNG.

PYTHONPATH=. ../RemoteTeleop/.venv/bin/python scripts/investigation/vehicle_shape/preset_gallery.py \
    --output ../RemoteTeleop/artifacts/pcd-vehicle-assets-20261015-v3/presets.png
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from src.viz.composited_camera import PyTorch3DObjectMeshRenderer
from src.viz.preview_car_proxy import _camera
from src.viz.universal_vehicle import TYPE_NAMES, UniversalVehicleShape
from src.viz.vehicle_proxy import VehicleProxyRenderer

VIEWS = (("front 3/4", (1.3, -1.6, .55)), ("side", (0., -1., .08)), ("rear 3/4", (-1.3, -1.6, .55)),
         ("rear, elevated", (-1.4, -.7, 1.3)))
TINTS = {"sedan": (.75, .12, .12), "hatchback": (.95, .65, .1), "crossover": (.15, .35, .75),
         "full_suv": (.12, .12, .14), "minivan": (.82, .83, .85), "van": (.95, .95, .95), "pickup": (.25, .45, .3)}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--box", type=json_lwh, default=None,
                        help='render means normalized into one box, e.g. "[5.0,2.2,1.87]", with CooperScene inflation')
    args = parser.parse_args()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    proxies = VehicleProxyRenderer(PyTorch3DObjectMeshRenderer(device=args.device))
    image_size = (640, 400)
    fig, axes = plt.subplots(len(TYPE_NAMES), len(VIEWS), figsize=(4.2 * len(VIEWS), 2.75 * len(TYPE_NAMES)),
                             facecolor="#f1f4f8", squeeze=False)
    for row, body_type in enumerate(TYPE_NAMES):
        if args.box is None:
            shape = UniversalVehicleShape.mean(body_type)
            dimensions = np.asarray(shape.prior.archetype_lwh)
        else:
            dimensions = np.asarray(args.box)
            shape = UniversalVehicleShape.mean(body_type, dimensions, inflation_lwh_m=(.3, .36, .32))
        model = np.eye(4)
        model[2, 3] = dimensions[2] / 2
        for column, (view, direction) in enumerate(VIEWS):
            camera, intrinsic = _camera(direction, dimensions, image_size)
            overlay = proxies.render("car", dimensions, model, camera, intrinsic, image_size, shape=shape,
                                     body_color_rgb=TINTS[body_type])
            alpha = overlay.alpha[..., None]
            rgb = overlay.rgb * alpha + np.array([.945, .957, .973]) * (1 - alpha)
            ax = axes[row, column]
            ax.imshow(np.clip(rgb, 0, 1))
            ax.axis("off")
            label = f"{body_type}  {dimensions[0]:.2f} x {dimensions[1]:.2f} x {dimensions[2]:.2f} m"
            ax.set_title(label if column == 0 else view, fontsize=10, loc="left")
    fig.suptitle("Universal vehicle mesh: type means (archetype sizes)" if args.box is None
                 else f"Universal vehicle mesh: type means in a {args.box} box", fontsize=15)
    fig.tight_layout(rect=(0, 0, 1, .975))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=90, facecolor=fig.get_facecolor())
    print(args.output)


def json_lwh(text: str):
    import json

    return json.loads(text)


if __name__ == "__main__":
    main()
