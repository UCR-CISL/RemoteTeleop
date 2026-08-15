# CooperScene localization and dynamic-object teleoperation handoff

Status: research prototype and benchmark checkpoint

Last updated: 2026-08-13

## Purpose

This document records the current CooperScene-to-Gaussian-splat localization,
camera rendering, SAM3/SAM3D dynamic-object overlay work, the measurements made
so far, and the intended next deployment architecture.

The current implementation is useful as an experiment harness, but it should
not be treated as the final vehicle/remote-operator software architecture. In
particular, the SAM3/SAM3D runtime has accumulated more orchestration,
residency modes, policies, and benchmark controls than should be carried into
the first deployment version. Simplifying that pipeline is an explicit next
task.

## Current system behavior

### Localization and coordinate frames

The pipeline retains CooperScene's global/ENU-like coordinate frame. Initial
localization estimates one fixed transform:

```text
gaussian_T_cooperscene
```

Per-frame poses then use the pose stored in the selected agent's YAML:

```text
gaussian_T_lidar = gaussian_T_cooperscene @ cooperscene_T_lidar
gaussian_T_camera = gaussian_T_lidar @ inverse(camera_T_lidar)
```

LiDAR is required for initial localization. The accepted single-agent result
is stored under:

```text
artifacts/cooperscene_single_agent_coarse_cuda/localization.json
```

After this initial transform is available, normal replay and rendering require
only:

- the per-frame YAML pose;
- the camera image path;
- fixed agent camera calibration;
- GT vehicle boxes and stable track IDs from the YAML.

They no longer open a PCD. PCD loading remains available for initial
localization and for an explicitly requested LiDAR overlap diagnostic.

### Full-take rendering

The current experiment uses CooperScene training take 1, agent 1:

- 501 ordered frames, IDs `481260` through `481760`;
- 10 Hz nominal source cadence;
- 50 seconds of dataset time;
- all source frames are processed by default;
- `--stop-at-overlap` is opt-in.

When `--stop-at-overlap` is absent, the renderer skips the LiDAR overlap
prepass and Gaussian-center KD-tree entirely. When it is supplied, the
front-camera-frustum LiDAR diagnostic remains available and may truncate the
take after sustained loss of Gaussian-map support.

### Dynamic objects

GT 3D vehicle boxes are transformed and projected into the camera. A projected
box proxy is available immediately and remains the authoritative real-time
fallback.

Tracks are admitted to SAM3 after sustained camera visibility. The current
defaults are:

- two seconds of stable visibility;
- short gaps tolerated up to 0.25 seconds;
- admission waves of up to five tracks;
- no requirement to wait until five tracks are present;
- tracks that disappear quickly retain only the box proxy.

SAM3 produces an object mask. SAM3D reconstructs one admitted object at a time.
If a mesh becomes ready while its track is still valid, the compositor replaces
the proxy with a metric-aligned mesh. Late and stale results are cancelled or
discarded rather than displayed at an incorrect pose.

The measured 24 GiB GPU configuration selected for continued experiments is:

```text
SAM3:                 FP16 on CUDA
SAM3D dense weights:  selective NF4
SAM3D sparse compute: FP16
SAM3D diffusion:      12 / 12 steps
GSplat downsample:    4
```

Selective NF4 is restricted to compatible dense `torch.nn.Linear` layers. It
does not attempt to replace SAM3D sparse convolution modules.

## Performance checkpoint

### PCD-backed replay baseline

The earlier deployment-candidate run eagerly parsed every ASCII PCD even though
replay, SAM3, SAM3D, and the compositor did not consume the point cloud.

```text
artifacts/cooperscene_runtime_followups_sol_20260813/
  sam3_fp16_sam3d_nf4_steps12_render4/
```

Key results:

| Metric | Result |
|---|---:|
| Published frames | 501 |
| Rendered frames | 499 |
| Composited output | 1.427 FPS |
| Median replay publish interval | 672.1 ms |
| Median compositor render | 145.9 ms |
| Ready SAM3D meshes | 9 |
| Peak GPU memory | 21,236 MiB |

### Metadata-only replay

The optimized run used the same model precision, SAM3D steps, render
downsample, dataset, localization, and admission configuration, but performed
no PCD reads after localization:

```text
artifacts/cooperscene_runtime_metadata_only_20260813/
```

Key results:

| Metric | Result |
|---|---:|
| Published frames | 501 |
| Rendered frames | 484 |
| Replay publication | 2.826 FPS |
| Composited output | 2.738 FPS |
| Median replay publish interval | 349.7 ms |
| Median compositor render | 145.3 ms |
| Ready SAM3D meshes | 2 |
| Peak GPU memory | 20,583 MiB |

This is a 91.9% composited-FPS improvement over the exact prior deployment
candidate, or approximately 83.7% relative to the rounded 1.49 FPS comparison
used during experimentation.

Every replay metric record reports:

```text
lidar_points_loaded = false
pcd_read_ms = 0.0
```

The new timing breakdown is:

| Per-frame stage | Median | P95 |
|---|---:|---:|
| YAML pose/box metadata | 8.62 ms | 14.02 ms |
| Camera PNG read/decode | 333.52 ms | 366.59 ms |
| Pose and box projection | 0.78 ms | 1.79 ms |
| JPEG encoding | 4.25 ms | 8.22 ms |
| Uninstrumented build residual | 0.21 ms | 0.67 ms |

Removing unused PCD parsing approximately halved total replay time, from 349.5
seconds to 176.9 seconds. Camera PNG filesystem access and decoding is now the
dominant replay cost.

The faster source also exposes a policy tradeoff: more tracks expire in wall
time before sequential SAM3D reconstruction finishes. Box proxies remain
correct, but only two meshes became ready in the faster run. Mesh yield should
therefore be evaluated separately from camera delivery FPS.

The new plots and summaries are:

```text
artifacts/cooperscene_runtime_metadata_only_20260813/timeline/
  dataset_time_timeline.png
  wall_clock_timeline.png
  fps_timeline.png
  timeline_summary.json
```

The rendered video is:

```text
artifacts/cooperscene_runtime_metadata_only_20260813/
  sam3_fp16_sam3d_nf4_steps12_render4/composited/composited.mp4
```

## Interpreting the current FPS

The 2.738 FPS result is an end-to-end result for the offline CooperScene PNG
replay implementation. It includes filesystem access, PNG decoding, projection,
JPEG re-encoding, transport, GSplat rendering, dynamic-object compositing, and
output writing.

It is not a direct prediction of live camera performance. `cv2.imread()` on a
1920x1200 lossless PNG is different from receiving a ROS image message:

- `sensor_msgs/msg/Image` normally provides a raw pixel buffer and does not
  require PNG/JPEG decompression;
- `sensor_msgs/msg/CompressedImage` requires the corresponding decoder;
- a camera driver, GStreamer pipeline, or hardware codec has its own buffering
  and decode behavior;
- ROS adds middleware serialization, delivery, QoS, callback scheduling, and
  queueing costs that direct file access does not reproduce.

Deployment measurements should continue to report three distinct rates:

1. source acquisition/callback FPS;
2. processing capacity once a frame is available;
3. source-to-operator composited FPS and latency.

At a 10 Hz sensor cadence, the system has a 100 ms per-frame budget if every
camera frame must be displayed. The current median GSplat/compositor time is
about 145 ms and the PNG replay source costs about 350 ms, so neither path yet
meets that requirement. A latest-frame policy remains appropriate for keeping
latency bounded, but dropped/superseded frames must remain visible in metrics.

## Next acquisition experiment: ROS 2 MCAP playback

The next source benchmark should use ROS 2 bag playback backed by MCAP. The
application should subscribe normally; it should not open or iterate through
the MCAP itself.

Conceptually:

```text
MCAP file -> ros2 bag player -> ROS/DDS topic -> application subscriber
```

This exercises the intended deployment-facing boundary: ROS serialization and
deserialization, QoS, callback scheduling, subscriber queues, and downstream
backpressure.

The recording should use the same camera message representation planned for
the vehicle:

- replay `sensor_msgs/msg/Image` if deployment publishes raw frames;
- replay `sensor_msgs/msg/CompressedImage` if deployment publishes compressed
  JPEG/PNG frames;
- preserve the real decoder when deployment uses H.264 or another encoded
  transport.

Example ROS 2 playback for a ROS-compatible MCAP is:

```bash
ros2 bag info -s mcap /path/to/recording.mcap
ros2 bag play -s mcap --rate 1.0 /path/to/recording.mcap
```

A generic MCAP containing non-ROS Protobuf, JSON, or custom schemas cannot be
published directly as ROS messages without a converter or bridge.

The ROS source adapter should record:

- message header/source timestamp;
- subscriber callback arrival using a monotonic wall clock;
- queue depth and sequence gaps;
- decode and color-conversion time, when applicable;
- time at frame publication to the rest of the pipeline;
- composited completion time.

If `/clock` and `use_sim_time` are used, dataset time and monotonic execution
time must still remain separate. Historical header timestamps must not be
subtracted directly from the current wall clock.

## Intended two-system deployment split

The deployment is planned as two systems.

### Vehicle system

The vehicle-side system owns sensor ingestion and dynamic-object mesh
production:

```text
camera / pose / detections
        -> tracking and stable admission
        -> SAM3 mask
        -> SAM3D mesh reconstruction
        -> metric alignment and mesh cache
        -> transmit lifecycle events and mesh assets
```

It should publish box/track state immediately and send meshes asynchronously.
The camera/control path must not block on SAM3 or SAM3D. Each mesh should be
generated once per `(scene_generation, track_id)`, cached, and referenced by a
stable asset ID.

The minimum outbound dynamic-object protocol needs:

- scene generation and stable track ID;
- object lifecycle state;
- authoritative metric dimensions and pose convention;
- mesh asset ID/version and content hash;
- mesh payload or asset-transfer reference;
- timestamps, status, and error information.

### Remote teleoperation system

The remote system owns the operator-facing visualization:

```text
camera / ego pose / boxes + mesh lifecycle stream
        -> Gaussian-splat camera render
        -> immediate projected box proxies
        -> remote mesh cache
        -> depth-aware mesh overlay when ready
        -> operator display
```

The remote viewer must remain usable when meshes are delayed, dropped, or
unavailable. Box proxies are the guaranteed representation; meshes are an
optional asynchronous enhancement.

The transport between systems still needs a concrete decision for reliability,
compression, retransmission, asset caching, and reconnect behavior. Camera and
control traffic should remain independent from large mesh transfers so an
asset cannot stall teleoperation.

## Simplification workstream

The current SAM3/SAM3D runtime is intentionally feature-rich for experiments,
but it is over-engineered for the first two-system deployment. Complexity now
includes multiple precision modes, resident and take-turn residency paths,
CPU capability gates, several admission/priority policies, process health
orchestration, benchmark variants, and multiple transport roles.

The simplification task should begin by preserving only the behavior proven to
be necessary:

1. one vehicle-side frame subscriber;
2. one stable-track admission component;
3. one resident SAM3 worker;
4. one sequential SAM3D worker with a bounded queue;
5. one mesh cache and publisher;
6. one remote receiver/cache/compositor;
7. immediate proxy rendering and explicit terminal asset states;
8. a small fixed metrics schema.

Recommended removals or separations:

- move experiment matrices and precision sweeps out of the runtime supervisor;
- remove destructive model reload/take-turn paths from the deployment path;
- replace loosely coupled CLI forwarding with one typed configuration per
  system;
- collapse benchmark-only policy combinations into offline experiment tools;
- keep one admission policy until measurements justify alternatives;
- define one transport interface for mesh lifecycle/assets instead of exposing
  internal worker protocol details to the remote viewer;
- retain failure isolation and metrics, but reduce worker-state machinery to
  the states the deployment actually consumes.

This should be a deliberate refactor after the vehicle/remote boundary and ROS
message contracts are written down. Simplification should not remove immediate
box proxies, stable IDs, stale-result rejection, or non-blocking camera/control
behavior; those are core correctness properties rather than experimental
complexity.

## Recommended next tasks

1. Define the deployment camera, pose, box, mesh lifecycle, and mesh asset ROS
   message contracts.
2. Record or convert a representative 10 Hz camera/pose/box sequence into a
   ROS-compatible MCAP using the intended raw or compressed camera type.
3. Add a ROS subscriber source adapter and replay the MCAP through
   `ros2 bag play`, measuring acquisition, processing, and operator-output FPS
   separately.
4. Split the current process graph into vehicle-side mesh production and
   remote-side GSplat/compositing executables.
5. Define mesh transport reliability, caching, versioning, reconnect, and
   bandwidth limits.
6. Simplify the SAM3/SAM3D orchestration to the minimal deployment path above.
7. Re-evaluate admission lifetime and SAM3D queue policy at the real 10 Hz
   source rate; faster replay currently reduces completed mesh yield.
8. Optimize or replace the remaining source and render bottlenecks only after
   MCAP measurements establish their live equivalents.

## Deployment success criteria

- sustain the real sensor acquisition rate, initially 10 Hz;
- keep camera/control delivery independent of SAM3D and mesh transport;
- bound operator-visible latency with latest-frame behavior and explicit drop
  metrics;
- display a correct proxy for every live tracked object immediately;
- never display a stale mesh for an expired or reused track ID;
- deliver and cache useful meshes asynchronously without blocking the viewer;
- preserve coordinate-frame and metric-pose conventions across both systems;
- expose enough stage timing to identify source, transport, rendering, and
  reconstruction bottlenecks independently.
