# Coordinate frames

The GSplat teleoperation code uses homogeneous transforms named
`target_T_source`. Multiplying a homogeneous point expressed in `source` by
`target_T_source` expresses it in `target`.

The canonical vehicle convention is right-handed, Z-up, X-forward, Y-left, in
meters. Public quaternions are ordered `[x, y, z, w]`; the nuScenes loader
converts the dataset's `[w, x, y, z]` records at its boundary. Vehicle
dimensions are always `[length, width, height]`.

For the initial nuScenes vertical slice, `world` is the nuScenes global frame.
The loader exposes absolute `world_T_ego`, `world_T_lidar`,
`world_T_camera`, and `world_T_object` transforms at the relevant sensor
exposure. Ego-relative localization is derived rather than integrated:

```text
ego_T_object = inverse(world_T_ego) * world_T_object
```

When a DriveStudio scene is trained, its fixed map frame will be registered
with one explicit `drivestudio_map_T_nuscenes_global` transform. That
registration must be applied to ego, camera, and object poses alike; the
static Gaussian map must never move to follow the ego vehicle.

SAM3D translation and scale are visual reconstruction hints only. The
authoritative object pose and dimensions come from the tracked metric 3D box.
