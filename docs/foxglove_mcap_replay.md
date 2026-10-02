# CooperScene and nuScenes Foxglove MCAP replay

`McapFrameReplay` automatically recognizes Foxglove protobuf recordings and
retains support for the previous ROS2 CDR recordings. The SAM simulation launcher
and adapter accept `--mcap-agent 1` (or `2`/`3` for CooperScene). The same agent is
used for renderer calibration. Agent 0 in the mounted CooperScene recordings has
LiDAR but no camera.

The source recordings under
`/mnt/bcc-data/websites/data.ucr.edu-datasets-cooperscene/mcap` are read directly;
no conversion or writes to that directory are required. The reader joins camera,
LiDAR, global boxes, frame labels and poses by sample order, validates counts and
timestamp skew, and preserves the original capture timestamp separately from the
zero-origin replay clock. It uses the existing CooperScene camera calibration,
which is not embedded in the mounted files. Global boxes provide the union of
tracked vehicles; heading-marker cubes are excluded. Camera-visible boxes are
projected for the existing analysis/render pipeline.

## nuScenes export

```sh
python -m src.data.processed_nuscenes_mcap \
  --processed-root /path/to/processed_10Hz/trainval \
  --output-root artifacts/nuscenes_10hz_foxglove_mcap
```

Foxglove is the default output; `--format ros2` retains the old export. Output
contains `/1_Camera` (PNG CompressedImage), `/1_LIDAR` (XYZI PointCloud),
`/1_Bbox`, `/bbox_global` (SceneUpdate), and `/tf` (FrameTransform). Boxes use
map coordinates, matching the mounted recordings. Track IDs and full supported
vehicle category names are preserved, including offscreen vehicles. MCAP
`teleop` metadata embeds intrinsics and map alignment; camera and LiDAR poses
are recorded every frame. The ego transform aliases LiDAR, as in the existing
processed nuScenes pipeline. Completed files are replaced atomically.

Do not pass a single expected sample count when exporting multiple scenes of
different lengths. For replay, use each recording's actual frame count. A GSplat
map and correct dataset-to-map alignment are still required for rendering.
For native CooperScene source coordinates, the stored localization transform
is identity when configuring the existing rebase arguments.

## Verified Lambda inputs

Lambda already has processed scenes `152`, `164`, `171`, `200`, `209`, `359`,
`529`, and `916` under
`/home/justin/bilateral-driving-repro/data/nuscenes/processed_10Hz/trainval`.
For nuScenes replay, select `--source-mode nuscenes_mcap` and the matching
nuScenes camera parameters so the remote renderer uses the correct intrinsics.
Their frame counts are respectively 191, 191, 196, 191, 191, 196, 201, 201.
Matching GSplat exports are under
`/home/justin/bilateral-driving-repro/ckpts/nuscenes_pretrained_checkpoints_6cams/<scene>/exports/background_40000.ply`.
These inputs suffice for conversion; raw nuScenes tables are not required.
The renderer host needs its own copy of the selected map.
