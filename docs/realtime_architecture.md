# Real-time dynamic-car MVP

The MVP uses four independent processes and strict version-one multipart ZMQ
messages. JSON carries IDs, timestamps, metric transforms, dimensions, scores,
and lifecycle state; JPEG/PNG payloads remain separate binary parts.

```text
nuScenes replay --PUB frames--> SAM 3.1 mask worker
       |                            |
       +------PUB frames----------> Rerun viewer
                                    |
                         DEALER reconstruction requests
                                    |
                                    v
                         ROUTER SAM 3D Objects worker
                                    |
                         PUB asset lifecycle/meshes
                                    |
                                    v
                              Rerun viewer
```

Replay publishes complete GT snapshots at native keyframe timestamps. The mask
worker uses latest-value input and reports discarded frames. By default it masks
only tracks that do not yet have a pending or acknowledged reconstruction. The
`interactive_batch` prompt mode encodes one image and decodes every candidate
box together; `serial_grounding` remains an experiment baseline. Frames with no
new candidates bypass SAM 3.1 and publish an explicit empty mask batch. Offline
masks are available only through the explicit `offline` backend.

The SAM 3D worker owns one resident model and one inference thread. Its ROUTER
socket remains responsive while inference runs. Requests are deduplicated by
`(scene_generation, track_id)`. The baseline orders by mask/visibility/projected
area quality. The deadline policy uses a short coalescing window and image-edge
motion history to avoid starting tracks likely to leave before a mesh can be
ready. Events move through `queued`, `running`, and `ready`, `failed`, or
`cancelled`.

Replay emits an explicit scene-end snapshot. Queued work is cancelled when its
track leaves the latest snapshot, new requests for expired tracks are rejected,
and an in-flight result is discarded if the track exits before inference ends.
A successful mesh therefore means `ready` while the track is still live; the
worker never keeps a stale track visible to hide missed latency.

Meshes remain object-local. `MeshAligner` applies robust uniform scale and
ground/center alignment against authoritative nuScenes metric dimensions.
`world_T_object` remains the authoritative localization transform; SAM 3D pose
predictions are diagnostics only.

The supervisor loads and dummy-warms SAM 3.1, then loads SAM 3D Objects while
SAM 3.1 remains resident, and gates replay on simultaneous `READY` health. The
batched SAM 3.1 path warms a four-box decode. SAM 3D synthetic mesh warmup is
deliberately disabled because dummy masks produced unbounded geometry during
earlier trials. It does not respond to OOM by changing resolution, offloading,
or swapping models. Every process emits JSONL latency, throughput, queue,
dropped-frame, deadline, cancellation, and CUDA-memory metrics.

SAM 3D Objects remains single-object because its public postprocessing path is
not batch-correct and one object already peaks near the 24 GiB device limit.
Safe optimization controls instead include 25/25, 12/12, or 8/12 diffusion-step
sweeps and an opt-in same-frame MoGe depth cache. The cache reuses only the
mask-independent RGB depth output; per-mask clipping and object preprocessing
still run independently.

Current limitations:

- Batched SAM 3.1 uses the upstream interactive instance decoder rather than
  the serial grounding decoder, so latency and mask acceptance must both be
  compared on real frames before making it the deployment default.
- The unified environment retains the proven SAM 3D Python 3.11/Torch 2.5.1
  stack. SAM 3.1 is pinned to the official source but this is below its README
  recommended Python 3.12/Torch 2.7 runtime and must be treated as an empirical
  compatibility result.
- DriveStudio training and static GSplat compositing remain TODOs.
- Live detection, tracking, odometry, and vehicle-side integration remain TODOs;
  nuScenes GT supplies the current replay detector/tracker and metric state.

Run the controlled policy matrix in a dedicated host tmux session, then inspect
`comparison.json` and each generation-specific `summary.json`:

```bash
MAX_JOBS=2 .venv/bin/python scripts/run_realtime_experiments.py \
  --dataroot /path/to/nuscenes --scene scene-0061
```
