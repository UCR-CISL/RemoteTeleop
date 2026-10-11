"""Native CooperScene checkpoint adapter for raw, single-scan XYZI recordings.

Mount the pinned, verified runtime model config and checkpoint at /native.
Unlike the nuScenes adapter, this model expects four features and intensity
normalized by the published CooperScene converter. Clients send raw intensity;
do not also pass pilot --intensity-scale when using this server config.
"""

_base_ = ["/native/cooperscene_native_runtime_candidate.py"]

test_pipeline = [
    dict(type="RuntimeFrontCameraInputs", point_feature_count=4,
         intensity_scale=1 / 65535),
    dict(type="ImageAug3D", final_dim=[256, 512], resize_lim=[.3, .3],
         bot_pct_lim=[0., 0.], rot_lim=[0., 0.], rand_flip=False,
         is_train=False),
    dict(type="PointsRangeFilter", point_cloud_range=[-72., -72., -5.,
                                                       72., 72., 3.]),
    dict(type="Pack3DDetInputs", keys=["points", "img"],
         meta_keys=("cam2img", "ori_cam2img", "lidar2cam", "cam2lidar",
                    "lidar2img", "img_aug_matrix", "lidar_aug_matrix",
                    "sample_idx", "box_type_3d", "box_mode_3d", "dataset",
                    "img_shape", "ori_shape")),
]

test_dataloader = dict(dataset=dict(pipeline=test_pipeline))
