"""CooperScene crop experiment on the unchanged nuScenes BEVFusion checkpoint.

The architecture/checkpoint remain the upstream nuScenes model.  The runtime
input transform supplies exactly one front image and its matching matrices.
"""

_base_ = [
    "/mmdetection3d/projects/BEVFusion/configs/"
    "bevfusion_lidar-cam_voxel0075_second_secfpn_8xb4-cyclic-20e_nus-3d.py"
]

custom_imports = dict(
    imports=["projects.BEVFusion.bevfusion"], allow_failed_imports=False)

# The full converted checkpoint supplies the image backbone weights; avoid a
# redundant network download of Swin initialization on every container start.
model = dict(img_backbone=dict(init_cfg=None))

# The server registers RuntimeFrontCameraInputs before this pipeline is built.
# Subsequent geometry/packing transforms intentionally use a one-view leading
# dimension for all camera matrices.
# Match the upstream resized height (900 * .48 = 1200 * .36 = 432),
# preserving more traffic/horizon in CooperScene 1920x1200 frames.
# This is an offline pilot variant; the default config remains unchanged.
test_pipeline = [
    dict(type="RuntimeFrontCameraInputs"),
    dict(type="ImageAug3D", final_dim=[256, 704], resize_lim=[0.36, 0.36],
         bot_pct_lim=[0.0, 0.0], rot_lim=[0.0, 0.0], rand_flip=False,
         is_train=False),
    dict(type="PointsRangeFilter", point_cloud_range=[-54.0, -54.0, -5.0,
                                                       54.0, 54.0, 3.0]),
    dict(type="Pack3DDetInputs", keys=["points", "img"],
         meta_keys=("sample_idx", "box_type_3d", "box_mode_3d",
                    "cam2img", "lidar2cam", "cam2lidar",
                    "lidar2img", "dataset", "img_shape", "ori_shape",
                    "img_aug_matrix")),
]

test_dataloader = dict(dataset=dict(pipeline=test_pipeline))
