# Teleoperation GSplat Viewer — Implementation Plan

## 1. Objective

Build a remote teleoperation visualization system in which:

- A static driving scene is reconstructed as a Gaussian splat using **DriveStudio**.
- Initial development and evaluation use the **nuScenes** dataset.
- The vehicle-side process streams ego pose and dynamic-car state over **ZeroMQ (ZMQ)**.
- Dynamic cars are represented initially by proxy geometry and later by meshes reconstructed with **SAM 3D Objects**.
- LiDAR-derived 3D boxes provide metric object scale and placement.
- The teleoperator machine focuses on rendering, state interpolation, asynchronous object reconstruction, and asset management.
- Ground-truth nuScenes ego poses and object tracks are used first. Estimated odometry and BEVFusion are introduced only after the viewer and networking pipeline are working.

The initial implementation does not include vehicle command execution, local safety logic, collision avoidance, or live vehicle integration.

---

## 2. Core Design Principles

1. **The DriveStudio scene is the fixed metric world.**

   The trained static Gaussian scene defines the world coordinate frame used by the viewer.

2. **Dynamic cars are not baked into the static background.**

   DriveStudio must be trained or exported with dynamic vehicles excluded from the background representation. Dynamic cars are rendered as independent scene nodes.

3. **The vehicle-side process owns authoritative motion state.**

   It publishes:

   - ego pose,
   - car track IDs,
   - car poses,
   - car velocities,
   - car dimensions,
   - timestamps,
   - confidence and lifecycle state.

4. **The teleoperator owns visual reconstruction and rendering.**

   It loads the static GSplat, interpolates streamed state, displays proxy cars immediately, and asynchronously replaces proxies with SAM 3D Object meshes.

5. **SAM 3D Object does not determine metric scale.**

   Its output is placed and scaled using LiDAR-derived car dimensions and poses.

6. **Rendering must never block on object reconstruction.**

   SAM 3D Object runs in a separate process, ideally on a separate GPU. The renderer continues using a proxy while reconstruction is pending.

7. **Pose and object-state streams are latest-value data.**

   Old poses should be discarded rather than queued and replayed after network stalls.

---

## 3. High-Level Architecture

```text
nuScenes / future live sensors
          |
          v
+-----------------------------------+
| Vehicle-Side Process              |
|                                   |
| Dataset replay or live ingestion  |
| GT ego pose -> later odometry     |
| GT boxes -> later BEVFusion       |
| Car tracking                      |
| LiDAR metric dimensions           |
| Keyframe selection                |
| ZMQ publishers                    |
+------------------+----------------+
                   |
                   | ZMQ
                   |
+------------------v----------------+
| Teleoperator Machine              |
|                                   |
| ZMQ receivers                     |
| World-state manager               |
| Pose interpolation                |
| DriveStudio GSplat renderer       |
| Proxy car renderer                |
| SAM 3D worker process             |
| Mesh scaling/alignment            |
| Asset cache                       |
+-----------------------------------+
```

---

## 4. Coordinate Frames

Use an explicit transform tree.

```text
world / drivestudio_map
├── ego_vehicle
│   ├── camera_front
│   ├── other_cameras
│   └── lidar
├── car_<track_id>
├── car_<track_id>
└── ...
```

The viewer receives absolute transforms:

```text
world_T_ego(timestamp)
world_T_car(track_id, timestamp)
```

Do not stream only incremental motion.

For the initial nuScenes implementation:

- `world` should match the processed DriveStudio scene coordinate frame.
- Ground-truth ego poses must be transformed into that same frame.
- Ground-truth object poses must use the same convention.
- Record all axis conventions explicitly:
  - handedness,
  - up axis,
  - forward axis,
  - quaternion ordering,
  - units.

All world positions and object dimensions must use meters.

---

## 5. Repository Structure

Use a structure similar to:

```text
teleop-gsplat/
├── README.md
├── pyproject.toml
├── configs/
│   ├── replay.yaml
│   ├── network.yaml
│   ├── viewer.yaml
│   └── reconstruction.yaml
├── proto/
│   └── teleop.proto
├── src/
│   ├── common/
│   │   ├── transforms.py
│   │   ├── timestamps.py
│   │   ├── messages.py
│   │   └── config.py
│   ├── vehicle/
│   │   ├── nuscenes_replay.py
│   │   ├── gt_state_provider.py
│   │   ├── bevfusion_provider.py
│   │   ├── tracker.py
│   │   ├── lidar_object_geometry.py
│   │   ├── keyframe_selector.py
│   │   └── zmq_server.py
│   ├── teleoperator/
│   │   ├── zmq_client.py
│   │   ├── world_state.py
│   │   ├── interpolation.py
│   │   ├── viewer.py
│   │   ├── drivestudio_adapter.py
│   │   ├── proxy_assets.py
│   │   ├── asset_manager.py
│   │   └── reconstruction_client.py
│   └── reconstruction/
│       ├── worker.py
│       ├── sam3d_runner.py
│       ├── mesh_alignment.py
│       ├── mesh_postprocess.py
│       └── cache.py
├── scripts/
│   ├── prepare_nuscenes.sh
│   ├── train_static_scene.sh
│   ├── replay_scene.sh
│   ├── run_viewer.sh
│   └── run_reconstruction_worker.sh
├── tests/
│   ├── test_transforms.py
│   ├── test_serialization.py
│   ├── test_interpolation.py
│   ├── test_replay.py
│   └── test_mesh_alignment.py
└── docs/
    ├── coordinate_frames.md
    ├── protocol.md
    └── drivestudio_integration.md
```

DriveStudio and SAM 3D Object may initially remain separate repositories or submodules. Avoid heavily modifying either upstream repository until the integration boundaries are understood.

---

# 6. Implementation Stages

## Stage 0 — Environment and Data Preparation

### Goal

Confirm that DriveStudio can process nuScenes and produce a renderable static scene.

### Tasks

- [ ] Clone DriveStudio with submodules.
- [ ] Create the required Python environment.
- [ ] Install DriveStudio dependencies, including the expected `gsplat` version.
- [ ] Download nuScenes `v1.0-mini`.
- [ ] Run DriveStudio preprocessing for one scene only.
- [ ] Inspect the processed scene structure:
  - images,
  - LiDAR,
  - ego/LiDAR poses,
  - camera intrinsics,
  - camera extrinsics,
  - object annotations,
  - dynamic masks.
- [ ] Document DriveStudio and nuScenes coordinate conventions.
- [ ] Select a short nuScenes mini scene with:
  - visible cars,
  - moderate ego motion,
  - sufficient static background,
  - no severe data gaps.
- [ ] Configure DriveStudio to reconstruct the static background while excluding dynamic cars.
- [ ] Train the static Gaussian scene offline.
- [ ] Render the trained scene from dataset camera poses.
- [ ] Determine the checkpoint format and identify:
  - Gaussian means,
  - scales,
  - rotations,
  - colors or spherical harmonics,
  - opacities,
  - scene normalization transforms.
- [ ] Decide whether to:
  - render directly through DriveStudio, or
  - export/load the static Gaussians through a custom viewer adapter.

### Deliverables

- A reproducible DriveStudio environment.
- One processed nuScenes scene.
- One trained static-background checkpoint.
- A script that renders the scene from known camera poses.
- `docs/coordinate_frames.md`.
- `docs/drivestudio_integration.md`.

### Acceptance Criteria

- The selected scene renders correctly using DriveStudio.
- Dynamic cars are not visibly baked into the static background, or their residual artifacts are documented.
- A known nuScenes camera pose maps to the expected location and orientation in the rendered scene.
- All transforms are metric and deterministic.

---

## Stage 1 — Local Dataset-Replay MVP

### Goal

Prove that streamed ego and car state can drive a coherent local visualization.

Do not use BEVFusion or SAM 3D Object yet.

### Vehicle-Side Tasks

- [ ] Implement a nuScenes replay process.
- [ ] Replay samples using dataset timestamps.
- [ ] Support configurable playback:
  - real time,
  - slowed down,
  - accelerated,
  - paused,
  - frame stepped.
- [ ] Read ground-truth ego pose.
- [ ] Read ground-truth car annotations.
- [ ] Use nuScenes instance tokens as stable track IDs.
- [ ] Filter dynamic objects to cars only.
- [ ] Convert ego and car poses into the DriveStudio world frame.
- [ ] Publish complete state snapshots locally through ZMQ.

### Teleoperator Tasks

- [ ] Load the trained static DriveStudio scene.
- [ ] Receive ego and car state.
- [ ] Place the virtual camera using the streamed ego pose.
- [ ] Create one scene node per car track.
- [ ] Render a generic car proxy for every active track.
- [ ] Scale proxies using GT 3D box dimensions.
- [ ] Remove proxies when tracks disappear.
- [ ] Display basic debug overlays:
  - track ID,
  - object dimensions,
  - age of latest state,
  - replay timestamp,
  - ego pose.

### Initial ZMQ Design

Use one PUB/SUB connection for state snapshots.

Topics:

```text
state/world
state/health
```

`state/world` should contain:

- protocol version,
- sequence number,
- timestamp,
- map ID,
- ego pose,
- ego velocity if available,
- complete active-car list.

A complete snapshot is preferred over incremental create/update/delete messages for the MVP because it automatically recovers from dropped PUB/SUB messages.

### Acceptance Criteria

- A nuScenes scene can be replayed end to end.
- The virtual camera follows the GT ego trajectory.
- Car proxies appear in correct metric locations.
- Proxy motion is spatially consistent with the static scene.
- Restarting the viewer does not require restarting the replay process.
- Dropped state messages do not permanently corrupt scene state.

---

## Stage 2 — Networked Viewer MVP

### Goal

Run dataset replay and rendering on separate machines.

### Tasks

- [ ] Run the replay process on the simulated vehicle machine.
- [ ] Run the viewer on the teleoperator machine.
- [ ] Use ZMQ over TCP.
- [ ] Add:
  - sequence numbers,
  - monotonic timestamps,
  - heartbeat messages,
  - protocol version,
  - map identifier,
  - reconnect behavior.
- [ ] Configure a small receive high-water mark.
- [ ] Drop stale state snapshots.
- [ ] Add a viewer-side jitter buffer.
- [ ] Interpolate ego and car poses between received states.
- [ ] Add bounded short-horizon extrapolation when the newest packet is delayed.
- [ ] Mark data stale after a configurable timeout.
- [ ] Log:
  - publish timestamp,
  - receive timestamp,
  - render timestamp,
  - network delay,
  - effective visualization latency,
  - dropped sequence count.

### Recommended State Rates

- GT replay source: up to 10 Hz using DriveStudio’s interpolated nuScenes data.
- Viewer rendering: 60 Hz or higher.
- Viewer interpolates between state samples.

### Acceptance Criteria

- Replay and viewer work across two machines.
- Viewer remains smooth when state arrives below render rate.
- Large queue buildup does not occur after a network pause.
- Viewer visibly indicates stale state.
- End-to-end latency is measurable.

---

## Stage 3 — Proxy Asset and World-State Hardening

### Goal

Build robust dynamic-car lifecycle management before introducing reconstruction.

### Tasks

- [ ] Implement a world-state manager keyed by `(vehicle_id, track_id, track_generation)`.
- [ ] Track:
  - current pose,
  - previous pose,
  - dimensions,
  - velocity,
  - confidence,
  - last update time,
  - asset state.
- [ ] Define asset states:
  - `PROXY`,
  - `RECONSTRUCTION_QUEUED`,
  - `RECONSTRUCTING`,
  - `READY`,
  - `FAILED`.
- [ ] Support multiple proxy car shapes or templates.
- [ ] Add deterministic cleanup of expired tracks.
- [ ] Add periodic full snapshots.
- [ ] Add a control socket for:
  - requesting current snapshot,
  - requesting replay reset,
  - requesting another keyframe,
  - querying protocol and map metadata.
- [ ] Separate sockets:
  - PUB/SUB for latest-value world state,
  - DEALER/ROUTER for reliable requests and acknowledgments.

### Acceptance Criteria

- Dynamic-car lifecycle is correct under track creation, disappearance, reconnection, and replay resets.
- Viewer never blocks waiting for an asset.
- Proxy rendering remains stable under packet loss and delayed messages.

---

## Stage 4 — SAM 3D Object Reconstruction Prototype

### Goal

Asynchronously reconstruct a car mesh from a selected nuScenes image and mask, then replace the proxy.

Keep GT ego poses and GT car tracks at this stage.

### Vehicle-Side Tasks

- [ ] Select one camera observation for a car.
- [ ] Extract:
  - RGB crop,
  - segmentation mask,
  - crop coordinates,
  - camera intrinsics,
  - camera extrinsics,
  - ego pose at exposure,
  - GT or LiDAR-derived car dimensions,
  - car pose,
  - optional associated LiDAR points.
- [ ] Implement a basic keyframe-quality score using:
  - mask area,
  - truncation,
  - occlusion,
  - image sharpness,
  - distance,
  - viewing angle.
- [ ] Send reconstruction requests over a reliable ZMQ socket.
- [ ] Cache requests until receipt acknowledgment.

### Reconstruction Worker Tasks

- [ ] Run in a separate process from the viewer.
- [ ] Load SAM 3D Object once at process startup.
- [ ] Accept reconstruction jobs through an IPC or ZMQ queue.
- [ ] Run single-image car reconstruction.
- [ ] Convert output to a renderer-friendly mesh representation.
- [ ] Normalize the mesh into an object-local coordinate frame.
- [ ] Estimate the predicted mesh bounding box.
- [ ] Scale the mesh using GT or LiDAR-derived dimensions.
- [ ] Prefer isotropic scaling initially.
- [ ] Align mesh orientation to the object coordinate convention.
- [ ] Ground the mesh using its lowest point or estimated wheel/ground contact.
- [ ] Simplify mesh geometry if required.
- [ ] Compress textures if required.
- [ ] Store result in an asset cache.
- [ ] Return asset readiness metadata to the viewer.

### Viewer Tasks

- [ ] Continue rendering a proxy while reconstruction runs.
- [ ] Atomically replace proxy with reconstructed mesh when ready.
- [ ] Preserve the authoritative streamed object pose.
- [ ] Do not use SAM 3D Object’s inferred world pose as authoritative.
- [ ] Allow reconstruction failure without affecting tracking or rendering.

### Acceptance Criteria

- At least one nuScenes car is reconstructed from a keyframe.
- Reconstruction runs outside the render process.
- Viewer frame rate does not stall during reconstruction.
- Completed mesh is placed at the correct metric position.
- Mesh dimensions approximately match the GT or LiDAR 3D box.
- The object continues moving according to streamed track poses.

---

## Stage 5 — Event-Driven Reconstruction and Bandwidth Policy

### Goal

Make keyframe transmission selective and bandwidth-aware.

### Default Policy

Prefer sending reconstruction inputs to the teleoperator:

- compressed RGB crop,
- compressed mask,
- calibration metadata,
- metric box,
- optional compact LiDAR points.

The teleoperator reconstructs and caches the mesh.

### Tasks

- [ ] Measure actual byte size of:
  - RGB crop,
  - mask,
  - LiDAR subset,
  - reconstructed mesh,
  - textures,
  - compressed GLB.
- [ ] Implement a per-track byte budget.
- [ ] Trigger reconstruction when:
  - a new important car track appears,
  - no reusable asset exists,
  - the current proxy is inadequate,
  - a substantially better keyframe is available.
- [ ] Avoid retransmitting keyframes after acknowledgment.
- [ ] Support at most one active reconstruction job per GPU initially.
- [ ] Prioritize by:
  - distance to ego,
  - projected screen size,
  - path relevance,
  - visibility quality,
  - reconstruction status.
- [ ] Add asset reuse within the same track.
- [ ] Record whether sending a generated mesh would have been cheaper than sending the reconstruction inputs.
- [ ] Keep vehicle-side mesh reconstruction out of scope unless measurements show a clear benefit.

### Acceptance Criteria

- Reconstruction requests are event-driven rather than frame-driven.
- Network use remains bounded.
- Duplicate requests are suppressed.
- Nearby relevant cars are reconstructed before distant cars.
- Viewer remains responsive under multiple simultaneous tracks.

---

## Stage 6 — Replace GT Cars with BEVFusion

### Goal

Replace ground-truth car boxes with BEVFusion while keeping GT ego pose.

This isolates perception errors from odometry errors.

### Tasks

- [ ] Integrate a reproducible BEVFusion inference pipeline on nuScenes.
- [ ] Restrict outputs to vehicle classes required by the project.
- [ ] Convert BEVFusion detections into the common world-frame schema.
- [ ] Add a multi-object tracker.
- [ ] Assign persistent track IDs.
- [ ] Estimate velocity from tracked poses.
- [ ] Associate detections with camera views.
- [ ] Obtain per-car masks:
  - use an existing segmentation model,
  - or derive masks from a dedicated instance-segmentation pipeline.
- [ ] Associate LiDAR points with each car using:
  - 3D box inclusion,
  - camera-mask projection,
  - or both.
- [ ] Estimate robust metric dimensions from LiDAR and tracked boxes.
- [ ] Smooth dimensions over time.
- [ ] Lock asset scale after confidence is sufficient.
- [ ] Compare against nuScenes GT:
  - 3D box translation error,
  - yaw error,
  - dimension error,
  - tracking continuity,
  - ID switches.

### Acceptance Criteria

- The viewer functions without GT car state.
- BEVFusion tracks remain visually aligned with the static scene.
- Car dimensions do not visibly pulse.
- Detection or tracking failures degrade to proxy removal rather than corrupting the viewer.
- Perception metrics are logged against GT.

---

## Stage 7 — Estimated Odometry on Recorded nuScenes

### Goal

Replace GT ego pose with estimated odometry while still using recorded data.

### Tasks

- [ ] Select an odometry implementation compatible with available nuScenes sensors.
- [ ] Run it offline or in replay mode.
- [ ] Produce timestamped `odom_T_ego`.
- [ ] Estimate the initial transform:

```text
world_T_odom
```

- [ ] Publish:

```text
world_T_ego = world_T_odom * odom_T_ego
```

- [ ] Compare estimated ego trajectory against nuScenes GT.
- [ ] Add pose covariance or confidence.
- [ ] Add support for relocalization or loop-closure corrections.
- [ ] Distinguish physical ego motion from map-frame correction.
- [ ] Smooth small visual corrections while applying authoritative pose immediately to state.

### Acceptance Criteria

- Viewer can run without GT ego pose.
- Estimated trajectory remains aligned with the DriveStudio map over a selected sequence.
- Drift and correction events are measurable.
- Viewer does not accumulate pose integration error from dropped network packets because it still receives absolute poses.

---

## Stage 8 — Live Vehicle Integration

### Goal

Replace nuScenes replay with live sensors.

### Tasks

- [ ] Integrate synchronized live cameras, LiDAR, IMU, and vehicle odometry.
- [ ] Replace dataset loader with a live sensor adapter.
- [ ] Preserve the same internal message schemas.
- [ ] Localize the live vehicle against a provided or prebuilt static GSplat map.
- [ ] Run BEVFusion and tracking on the vehicle GPU.
- [ ] Run keyframe selection on the vehicle.
- [ ] Stream state and reconstruction requests using the existing ZMQ protocol.
- [ ] Validate coordinate calibration end to end.
- [ ] Add operational monitoring:
  - sensor status,
  - localization status,
  - perception status,
  - network latency,
  - stale-state warnings,
  - reconstruction queue length,
  - render FPS.

Local safety, command validation, and vehicle-control execution remain separate future work.

---

# 7. ZMQ Protocol

## 7.1 Socket Layout

### State socket

```text
Vehicle PUB -> Teleoperator SUB
```

Use for:

- ego pose,
- active car snapshots,
- velocities,
- dimensions,
- health telemetry.

Properties:

- latest value matters,
- old messages may be dropped,
- small high-water mark,
- no retransmission.

### Reliable transfer socket

```text
Vehicle DEALER <-> Teleoperator ROUTER
```

Use for:

- reconstruction requests,
- keyframe receipt acknowledgments,
- asset status,
- snapshot requests,
- resend requests,
- protocol negotiation.

### Optional control socket

```text
Teleoperator DEALER <-> Vehicle ROUTER
```

Use for:

- replay pause/resume,
- frame stepping,
- reset,
- request-new-keyframe,
- diagnostics.

This may be combined with the reliable transfer socket initially.

---

## 7.2 Common Header

Use Protobuf or another schema-versioned binary format.

```protobuf
message Header {
  uint32 protocol_version = 1;
  string vehicle_id = 2;
  uint64 sequence_number = 3;
  uint64 timestamp_ns = 4;
  string frame_id = 5;
  string map_id = 6;
}
```

Do not use wall-clock receive time as the measurement timestamp.

---

## 7.3 Suggested Messages

```protobuf
message Pose {
  repeated double translation = 1; // x, y, z
  repeated double quaternion = 2;  // qx, qy, qz, qw
}

message Twist {
  repeated double linear = 1;
  repeated double angular = 2;
}

message CarState {
  uint64 track_id = 1;
  uint32 track_generation = 2;
  Pose world_pose = 3;
  Twist world_twist = 4;
  repeated float dimensions = 5; // length, width, height
  float confidence = 6;
  string asset_id = 7;
}

message WorldState {
  Header header = 1;
  Pose world_T_ego = 2;
  Twist ego_twist = 3;
  repeated CarState cars = 4;
}

message ReconstructionRequest {
  Header header = 1;
  uint64 request_id = 2;
  uint64 track_id = 3;
  uint32 track_generation = 4;
  bytes image = 5;
  bytes mask = 6;
  bytes lidar_points = 7;
  repeated float dimensions = 8;
  repeated double camera_intrinsics = 9;
  Pose world_T_camera = 10;
  Pose world_T_object = 11;
}
```

Large binary data should use multipart ZMQ frames:

```text
Frame 0: topic/message type
Frame 1: serialized metadata
Frame 2: compressed RGB
Frame 3: compressed mask
Frame 4: optional LiDAR payload
```

---

# 8. Interpolation and Rendering

The viewer should not move the static Gaussian map.

At render time:

- keep the DriveStudio world fixed,
- update the virtual camera pose from interpolated ego state,
- update each car scene node from interpolated car state.

Use:

- linear interpolation for translation,
- SLERP for rotation,
- bounded constant-velocity extrapolation only when necessary.

Do not extrapolate indefinitely.

Suggested initial values:

```yaml
state_buffer_ms: 100
max_extrapolation_ms: 150
stale_warning_ms: 300
track_expiration_ms: 1000
```

These values must remain configurable.

---

# 9. Metric Alignment of SAM 3D Meshes

For a reconstructed mesh with predicted dimensions:

```text
predicted = [length_hat, width_hat, height_hat]
measured  = [length, width, height]
```

Start with isotropic scale:

```text
scale = median(
    length / length_hat,
    width  / width_hat,
    height / height_hat
)
```

Then:

1. center the mesh in object-local coordinates,
2. align its forward/up axes to the project convention,
3. apply isotropic scale,
4. place its lowest point on the estimated ground plane,
5. attach the mesh to the streamed car scene node,
6. keep scale fixed for the remainder of the track unless confidence materially improves.

Do not update mesh scale every frame.

Anisotropic scaling may be added later but should not be the default because it can visibly deform cars.

---

# 10. DriveStudio Requirements

DriveStudio is used only for offline static-scene reconstruction in the target architecture.

Required behavior:

- dynamic vehicles are masked or excluded from the static background,
- output remains metric,
- camera poses remain aligned with nuScenes,
- a trained checkpoint can be loaded by the teleoperator renderer,
- the renderer can accept arbitrary camera poses,
- dynamic external scene nodes can be composited with the background.

Do not depend on DriveStudio’s offline per-vehicle Gaussian optimization for live dynamic cars.

Potential integration paths:

1. Keep DriveStudio’s renderer and add external mesh rendering.
2. Write a checkpoint adapter into a custom `gsplat` viewer.
3. Export static Gaussians to a portable representation if DriveStudio exposes enough checkpoint information.

Choose the least invasive option that produces correct rendering.

---

# 11. Non-Goals for the Initial Project

The following are intentionally out of scope until the visualization system works:

- vehicle command transmission,
- steering or braking control,
- local safety validation,
- collision avoidance,
- geofencing,
- production security,
- multi-operator arbitration,
- live map creation,
- real-time DriveStudio optimization,
- real-time Gaussian reconstruction of dynamic cars,
- pedestrian or cyclist reconstruction,
- dynamic-object classes other than cars.

---

# 12. Testing Strategy

## Unit Tests

- transform composition and inversion,
- quaternion ordering,
- frame conversion,
- timestamp ordering,
- serialization round trips,
- interpolation,
- stale-state detection,
- track lifecycle,
- mesh scale calculation.

## Integration Tests

- replay -> ZMQ -> viewer on one machine,
- replay and viewer on separate machines,
- viewer restart during active replay,
- packet loss and delayed packets,
- reconstruction worker failure,
- malformed or missing asset,
- track ID reuse,
- replay reset,
- map ID mismatch.

## Visual Validation

Render debug overlays for:

- GT box wireframes,
- streamed proxy geometry,
- reconstructed SAM 3D mesh,
- object-local axes,
- world axes,
- camera frustum,
- LiDAR points associated with a car.

## Metrics

Record:

- render FPS,
- SAM 3D reconstruction time,
- GPU memory by process,
- state message size,
- reconstruction request size,
- mesh asset size,
- network delay,
- visualization latency,
- dropped state messages,
- track ID switches,
- object translation error,
- dimension error,
- ego-pose error after odometry integration.

---

# 13. MVP Completion Definition

The MVP is complete when:

1. One nuScenes scene has been reconstructed as a static DriveStudio Gaussian map.
2. A separate replay process streams GT ego pose and GT car states through ZMQ.
3. A separate viewer loads the static GSplat.
4. The virtual camera follows the streamed ego trajectory.
5. Generic car proxies move correctly in the map using streamed car poses.
6. Rendering is smooth through interpolation.
7. Restarting either side does not permanently corrupt state.
8. All transforms and units are documented.

SAM 3D Object, BEVFusion, and estimated odometry are not required for MVP completion.

---

# 14. Medium Implementation Completion Definition

The medium implementation is complete when:

1. The MVP is stable across two machines.
2. A separate SAM 3D Object worker reconstructs selected cars.
3. LiDAR or GT 3D dimensions metrically scale the reconstructed mesh.
4. The viewer swaps proxies for meshes without blocking rendering.
5. Reconstruction requests are event-driven and acknowledged.
6. BEVFusion replaces GT car detections.
7. A tracker provides persistent IDs.
8. Perception outputs are evaluated against nuScenes GT.
9. GT ego pose is still allowed.

---

# 15. Final Recorded-Dataset Implementation Definition

The recorded-dataset implementation is complete when:

1. BEVFusion and tracking provide all dynamic-car state.
2. SAM 3D Object provides dynamic-car visual assets.
3. Estimated odometry replaces GT ego pose.
4. Ego pose is aligned with the DriveStudio map.
5. Reconnection and state resynchronization work.
6. Loop-closure or relocalization corrections are handled.
7. Latency, bandwidth, rendering performance, perception accuracy, and pose accuracy are logged.
8. The same interfaces can later accept live vehicle sensors without redesigning the system.

---

# 16. Recommended Immediate Work Order

Codex should proceed in this order:

1. Inspect DriveStudio’s nuScenes preprocessing and training configuration.
2. Identify how to train a static-background-only scene.
3. Reproduce rendering for one nuScenes mini scene.
4. Document coordinate-frame conventions.
5. Create the shared message schema.
6. Implement local GT dataset replay.
7. Implement a minimal ZMQ subscriber.
8. Build the DriveStudio viewer adapter.
9. Render GT car proxies.
10. Add interpolation and network separation.
11. Add the asynchronous SAM 3D worker.
12. Add LiDAR/GT metric mesh scaling.
13. Integrate BEVFusion.
14. Integrate estimated odometry last.

Do not begin BEVFusion, SAM 3D Object, or odometry integration until the previous stage has a working acceptance test.
