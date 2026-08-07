# First nuScenes Scene CUDA Benchmark

Run date: 2026-07-30
Dataset: nuScenes mini, `scene-0061`
GPU: NVIDIA GeForce RTX 4090 (24 GiB)
SAM 3D Objects commit: `81a82373a3a7f4cbb00bd5b32aaf6b4d0f659ddd`

## Result

The runner scanned all 39 frames, found 11 unique visible `vehicle.car`
tracks, selected one keyframe per track, and produced 11 raw and 11
metric-aligned GLBs. All 11 reconstruction attempts completed.

| Metric | Result |
|---|---:|
| Scene scan rate | 11.72 FPS |
| Mask preparation, mean / p50 / p95 | 0.686 / 0.144 / 3.122 s |
| Warm SAM 3D model load | 22.83 s |
| SAM 3D inference, mean / p50 / p95 | 6.94 / 6.36 / 9.65 s |
| Aggregate inference throughput | 0.144 objects/s |
| Reconstruction end-to-end, mean / p50 / p95 | 9.29 / 6.63 / 21.49 s |
| Warm reconstruction throughput, excluding first model load | 0.145 objects/s |
| PyTorch inference peak allocated / reserved | 18.58 / 22.88 GiB |
| `nvidia-smi` maximum device memory | 23.50 GiB |
| Active GPU utilization, mean | 73.75% |
| GPU temperature / power maximum | 72 C / 437.53 W |
| Aligned mesh payload total | 126.5 MB |

The event-driven reconstruction worker is not a per-frame real-time stage at
this quality setting. The vehicle-side scan and tracking path remains
independent; meshes are generated once per new track and cached.

## Metric Placement

nuScenes GT 3D annotations are authoritative for dimensions and pose. Every
manifest stores `world_T_object` and `ego_T_object` in meters, and the aligned
mesh remains object-local for attachment to that streamed pose. SAM 3D's
predicted translation, rotation, and scale are retained as diagnostics only.

The current plan-prescribed isotropic scale preserves shape. Across these 11
cars, mean absolute length/width/height fit residuals were
`[0.250, 0.502, 0.095]` m. The corresponding mean relative residuals were
`[5.30%, 26.24%, 5.94%]`. These are fit residuals against the same GT box used
for alignment, not independent perception accuracy. Poor-shape candidates
should be quality-gated before display; anisotropic correction remains deferred
because it visibly deforms meshes.

## Outputs

The complete run is under:

```text
artifacts/sam3d-real-first-scene-all/scene-0061/
```

Each selected track directory contains:

```text
image.jpg
mask.png
manifest.json
raw.glb
raw.sam3d.json
raw.aligned.glb
```

Scene-level timing and per-track metrics are in `metrics.json`; per-frame and
per-reconstruction events are in `events.jsonl`. One-Hz device telemetry is in:

```text
artifacts/sam3d-real-first-scene-all-gpu.csv
```

## Mesh-Only Follow-Up

The adapter now omits both Gaussian decoders and exports the mesh decoder's
vertex colors directly. A real follow-up reconstruction produced a watertight
284,886-vertex, 569,772-face GLB. It measured:

| Metric | Mesh-only result |
|---|---:|
| Model load | 21.99 s |
| Inference | 10.27 s |
| PyTorch peak allocated / reserved | 17.61 / 22.16 GiB |
| `nvidia-smi` maximum device memory | 23.30 GiB |

This saves some headroom but does not materially improve latency. Future
optimization should target generator inference steps, model quantization, or a
resident worker pool—not the mesh export path.

## Online SAM 3.1 Co-residency Admission

A real process-isolated run used the pinned SAM 3.1 multiplex checkpoint for
online box-prompted masks while the mesh-only SAM 3D Objects worker remained
resident. Both models loaded successfully and SAM 3.1 completed three warmup
passes before replay began.

| Metric | Online result |
|---|---:|
| Co-resident READY allocated memory, SAM3D / SAM 3.1 | 12.73 / 3.42 GiB |
| Co-resident READY reserved memory, SAM3D / SAM 3.1 | 12.79 / 4.83 GiB |
| Replay frames before terminal admission failure | 20 |
| Replay cadence | 2.09 FPS cumulative |
| SAM 3.1 batches / prompts / accepted masks | 15 / 87 / 87 |
| SAM 3.1 stale input frames discarded | 5 |
| SAM 3.1 inference mean / p50 / max | 615 / 470 / 1459 ms |
| SAM 3.1 batch effective FPS, mean | 2.13 |
| Unique reconstruction requests acknowledged | 11 |
| SAM3D first inference before OOM | 9.45 s |
| SAM3D inference peak allocated / reserved | 16.86 / 17.56 GiB |

The models fit simultaneously at idle and online SAM 3.1 ran at approximately
the nuScenes keyframe cadence. The first concurrent SAM3D decode then failed
while requesting another 512 MiB: only 258.62 MiB device memory remained.
Per the admission policy, the supervisor recorded the OOM and stopped; it did
not lower resolution, offload, quantize, or swap models. No mesh was completed
in this co-resident run.

The complete failure report and process metrics are under:

```text
artifacts/realtime-scene-0061-final/
```

## FP16 Online End-to-End Run

The same isolated-process pipeline completed after enabling FP16 for both
models and CUDA expandable segments. Replay still uses online SAM 3.1 masks;
no offline masks were supplied.

| Metric | FP16 online result |
|---|---:|
| Replay frames | 39 |
| SAM 3.1 processed / discarded frame batches | 31 / 8 |
| SAM 3.1 prompts accepted | 93 |
| SAM 3.1 inference mean / p50 | 430 / 274 ms |
| SAM 3.1 mean inference rate | 2.32 FPS |
| Unique reconstruction requests | 11 |
| Completed / failed meshes | 11 / 0 |
| SAM3D inference mean / p50 / range | 6.84 / 6.65 / 5.29–9.06 s |
| Aggregate SAM3D throughput | 0.146 objects/s |
| Maximum reconstruction queue latency | 65.95 s |
| SAM3D peak allocated / reserved | 17.35 / 17.50 GiB |

Each new track produced its own raw and metric-aligned GLB, for 22 GLBs total.
The complete artifacts and JSONL metrics are under:

```text
artifacts/realtime-scene-0061-fp16-v4/
```

This validates the intended asynchronous behavior, but it is not a real-time
success result: most meshes became ready only after their tracks had exited.
The current runtime cancels queued stale jobs and discards late in-flight
results instead of draining and displaying them. Box proxies remain the
immediate live representation. Some candidates also have large
length/width residuals: mean absolute `[length, width, height]` residual was
`[1.249, 0.307, 0.207]` m. A mesh quality gate remains required before
deployment.
