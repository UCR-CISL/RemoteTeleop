# ENU, GSplat, and LiDAR alignment

This note summarizes how the current CooperScene and NuScenes playback
pipelines relate global vehicle poses to their Gaussian-splat maps, and when
SE(3) or Sim(3) registration is appropriate.

## Transform convention

Transforms are named `target_T_source`. A point in `source` is expressed in
`target` by left-multiplying it by `target_T_source`.

The general runtime architecture is:

```text
1. Obtain one fixed transform between the global/ENU frame and the GSplat:
   gaussian_T_global

2. Follow the absolute vehicle trajectory in the global/ENU frame:
   global_T_ego(t)

3. Express every pose in the GSplat frame:
   gaussian_T_ego(t) = gaussian_T_global @ global_T_ego(t)
```

The vehicle is not independently realigned to the GSplat on every frame. A
single initial registration is followed by absolute global poses transformed
through that fixed registration.

## CooperScene

CooperScene supplies LiDAR and object poses in a common global, metric,
Z-up, ENU-like coordinate system:

```text
cooperscene_T_lidar(t)
cooperscene_T_object(t)
```

Initial localization registers one or more LiDAR scans to Gaussian centers and
estimates one fixed transform:

```text
gaussian_T_cooperscene
```

Normal replay then uses:

```text
gaussian_T_lidar(t)
    = gaussian_T_cooperscene @ cooperscene_T_lidar(t)

gaussian_T_camera(t)
    = gaussian_T_lidar(t) @ inverse(camera_T_lidar)

gaussian_T_object(t)
    = gaussian_T_cooperscene @ cooperscene_T_object(t)
```

For the CooperScene map tested so far, the two frames were already
approximately metric and aligned. The final registration was close to
identity, with roughly 0.736 m of translation and 0.993 degrees of rotation.
That correction is still important for accurate boxes and meshes even though
it is small relative to the map.

CooperScene rendering also applies a separate flat-ground correction. It
shifts the lowest oriented box corner onto the configured GSplat ground height
(currently `z = -2.8 m`). Initial registration and bbox grounding are therefore
related but distinct operations.

## NuScenes

The current Bilateral Driving NuScenes GSplat is not ENU-native. Training
anchors the map to the first front-camera pose. The fixed registration is
known from that convention rather than estimated with ICP:

```text
gaussian_T_global = inverse(global_T_first_front_camera)
```

The loader applies it consistently:

```text
gaussian_T_camera(t) = gaussian_T_global @ global_T_camera(t)
gaussian_T_lidar(t)  = gaussian_T_global @ global_T_lidar(t)
gaussian_T_object(t) = gaussian_T_global @ global_T_object(t)
```

BEVFusion predicts boxes in the LiDAR frame. The adapter places them into the
GSplat frame with:

```text
gaussian_T_object(t)
    = gaussian_T_lidar(t) @ lidar_T_object(t)
```

In processed NuScenes MCAPs, the compatibility field `world_T_ego` currently
contains `gaussian_T_lidar`. The exact `gaussian_T_camera` is carried
separately and is what the remote renderer uses.

Because the NuScenes GSplat is camera-anchored, physical vertical is not its Z
axis. The CooperScene fixed-world-Z grounding operation therefore cannot be
applied safely to this map. A correct grounding operation must move a box
along physical LiDAR vertical and use a local road-plane estimate.

## If the GSplat is generated in ENU

If the Gaussian map and vehicle localization use exactly the same metric ENU
origin, axes, and altitude datum, then:

```text
gaussian_T_ENU = identity
```

The runtime transforms simplify to:

```text
ENU_T_lidar(t)  = ENU_T_ego(t) @ ego_T_lidar
ENU_T_camera(t) = ENU_T_ego(t) @ ego_T_camera
ENU_T_object(t) = ENU_T_lidar(t) @ lidar_T_object(t)
```

In practice, a small fixed `gaussian_T_ENU` may remain because of calibration
or map-construction error. LiDAR-to-GSplat registration can estimate or verify
that residual correction, but it no longer needs to establish the entire
coordinate relationship.

An ENU-native map also makes physical vertical coincide with map Z. Grounding
can then shift boxes along Z to a local `z_ground(x, y)`. A constant ground Z
is sufficient only for an approximately flat road and a shared altitude
datum.

The GSplat must be fully ENU, not just have its Gaussian centers translated:
Gaussian orientations, scales/covariances, camera poses, metric scale, and all
other map-space geometry must use the same basis.

## SE(3) versus Sim(3)

The current LiDAR-to-GSplat registration is **SE(3)**:

```text
p_gaussian = R @ p_lidar + t
```

It estimates three rotation and three translation degrees of freedom. Scale is
fixed to one. Both coarse point-to-point registration and point-to-plane ICP
are configured as rigid estimators.

A **Sim(3)** registration additionally estimates one global scale:

```text
p_gaussian = s * R @ p_lidar + t
```

A GSplat reconstructed only from arbitrary-scale monocular structure from
motion can require Sim(3). Putting the LiDAR cloud into ENU does not, by
itself, make such a GSplat metric.

The intended separation is:

```text
Map construction:
    arbitrary-scale GSplat
        -> optional one-time Sim(3) calibration
        -> metric, preferably ENU-native GSplat

Runtime localization:
    metric ENU vehicle pose
        -> fixed or small SE(3) calibration
        -> GSplat-frame vehicle, camera, and object poses
```

If a one-time Sim(3) conversion is required, its scale must be applied to
Gaussian centers, Gaussian scales/covariances, camera translations, and other
map geometry. Runtime scan matching should then remain SE(3); it should not
continuously change the scale of the map.

The current CooperScene registration succeeding with low residuals under a
rigid transform is evidence that its LiDAR and GSplat already have compatible
metric scale. If their scales differed materially, SE(3) ICP would align one
region while producing growing residuals elsewhere.

## Practical conclusion

For a single playback, a good LiDAR-to-GSplat SE(3) registration can make a
separately supplied ENU-to-GSplat transform operationally redundant. An
ENU-native map remains valuable because it provides a stable shared frame for
multiple vehicles, repeat visits, telemetry, map updates, initialization, and
relocalization, while reducing or eliminating global scan registration at
runtime.
