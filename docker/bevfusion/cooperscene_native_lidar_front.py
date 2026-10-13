"""Native CooperScene LiDAR-only checkpoint adapter (bevfusion_single_lidar.pth).

Mount tracking-native-lidar-only-checkpoint-20261011 at /native. Same raw XYZI
wire input and server-side /65535 intensity scaling as the camera adapter; the
wire image is accepted but not packed, so camera bytes do not reach the model.
"""

_base_ = ["/native/cooperscene_native_lidar_runtime_candidate.py"]

test_pipeline = [
    dict(type="RuntimeFrontCameraInputs", point_feature_count=4,
         intensity_scale=1 / 65535),
    dict(type="PointsRangeFilter", point_cloud_range=[-72., -72., -5.,
                                                       72., 72., 3.]),
    dict(type="Pack3DDetInputs", keys=["points"],
         meta_keys=("box_type_3d", "sample_idx", "num_pts_feats")),
]

test_dataloader = dict(dataset=dict(pipeline=test_pipeline))
