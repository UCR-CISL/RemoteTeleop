# Split-system replay handoff: retry 14

## Scope and baseline

This is a **data-playback** split deployment: alien4 is the vehicle-side replay
host and alien3 is the remote-teleop compositor host.  It is not a real-vehicle
launch procedure.

The committed baseline is `c5b51f9` (`feat: enable real-time concurrent split
deployment`).  The resident-SAM changes used for retry 14 are intentionally
still uncommitted at this handoff:

- `scripts/deployment/run_remote_teleop_sam_simulation.py`
- `src/realtime/mask_process.py`
- `src/realtime/sam3d_worker.py`
- their focused tests and related documentation

There is also an unrelated supervisor reconciliation preserved in
`stash@{0}: wip: reconcile realtime supervisor CLI`; do not apply it as part of
the mesh-rendering fix.

## Current deployment behavior

The direct-MCAP adapter replays 501 source samples at 10 Hz.  It writes two
durable vehicle-local streams:

- an ordered pose/bbox render-snapshot spool, served to alien3 for GSplat and
  bbox-proxy rendering;
- an image-bearing analysis spool, consumed locally by SAM3/SAM3D.

The remote does not depend on image bytes for GSplat.  It derives the camera
pose from the received ego pose and its locally configured camera calibration.
Mesh delivery is a separate durable asset path.

The current launcher prewarms before starting either external stream endpoint:

1. start SAM3D with selective NF4 and retain it on CUDA;
2. wait for its durable ready marker;
3. start SAM3 in FP16 and retain it on CUDA;
4. wait for its durable ready marker;
5. start alien3's compositor, then alien4's adapter/direct MCAP replay.

Both models are CUDA-resident for the replay.  The shared lease serializes
**inference only**; it does not CPU-offload either resident model between
jobs.  This prewarm ordering is deliberate: it prevents remote frame requests
from accumulating while vehicle inference services initialize.

Launch from the repository root (choose new, non-existing artifact names for a
new run):

```bash
.venv/bin/python scripts/deployment/run_remote_teleop_sam_simulation.py \
  --config cfg/alien4_alien3.yaml \
  --output-root artifacts/split_system_sam_alien4_alien3_YYYYMMDD_retryNN \
  --local-artifacts artifacts/split_system_sam_alien4_alien3_YYYYMMDD_retryNN
```

The script defaults to the CooperScene MCAP, 501 expected samples, SAM3 FP16,
and SAM3D NF4.  Before a retry, verify no stale adapter/compositor/SAM worker
still owns the configured ports or PID/process groups.  Retry 12 was invalid
because a stale adapter owned port 8768; retry 13 exposed an old startup-order
failure.  Do not reuse an existing artifact directory.

## Retry 14: validated facts

Artifact root:
`artifacts/split_system_sam_alien4_alien3_20260819_retry14/`

- Remote MP4:
  `composited/composited.mp4`
- SHA-256:
  `de043f53ff1231c566c85e1cdbd34256f54e94d4f698d0f230c36050ab9a3f46`
- `frame-complete.json` records final sequence 500 and timestamp 50,000,000 us
  for scene `cooperscene:train:1:1`.
- The compositor recorded all 501 sequences (0--500) with zero sequence and
  timestamp gaps.  The output is 480x300, 10 FPS, 50.1 seconds.
- SAM3 and SAM3D prewarmed and completed without a recorded CUDA OOM.
- SAM3D produced ready meshes for tracks 12 and 104.  Their first-detection to
  ready-mesh times were 13.973 s and 21.575 s, respectively.  The reconstruction
  portions were 11.862 s and 7.363 s.
- Both corresponding GLBs were durably present on alien3 in the remote cache:
  - `composited/mesh-cache/cooperscene_train_1_1/cooperscene_train_1_1_12-v1-a77010c7958a978c79cf827c81fde13fb4f2c2f8cc494717d2881824afb92a14.glb`
  - `composited/mesh-cache/cooperscene_train_1_1/cooperscene_train_1_1_104-v1-1ee36895ebde71cc37c9a22aa82901e56e7849abbbca1ef8355ad736796a487b.glb`

## Open defect: remote mesh activation/rendering

Despite the two remote-cache GLBs, every retry-14 compositor metric reports
`mesh_count: 0`.  Bbox proxies continued rendering, including after the meshes
were available.  Therefore this is not evidence of a dropped pose/bbox frame,
late mesh transfer, or an inference-memory failure.

The root cause has since been reproduced on alien3: its `coop3r-slam`
environment does not provide `pytorch3d`.  The remote
`PyTorch3DObjectMeshRenderer.load` raises `ModuleNotFoundError` before parsing
the GLB.  Retry 14 silently converted that exception to a failed asset and kept
the proxy active.

The GLBs themselves are valid.  `trimesh` parses the retry-14 track-12 asset as
290,826 vertices / 581,632 faces and track 104 as 218,084 vertices / 436,212
faces.  Earlier artifact
`artifacts/cooperscene_runtime_both_gpu_nf4_v6_20260813/both_gpu_nf4/` records
the same `PyTorch3DObjectMeshRenderer` replacing proxies for tracks 12 and 104.
This is therefore an environment/activation failure, not a transport or mesh
format regression.

## Local fix prepared after retry 14

The current uncommitted worktree adds the following safeguards:

- the SAM launcher preflights alien3's configured Python for CUDA, `trimesh`,
  and the PyTorch3D renderer before starting either stream;
- the remote backend records `loaded`, `load_failed`, and `render_failed`
  outcomes instead of silently swallowing activation/raster exceptions;
- the compositor writes those outcomes as durable `compositor_mesh_asset`
  JSONL records with scene generation, track ID, request ID, path, and exception
  type/message;
- a failed mesh still leaves its bbox proxy active, so GSplat pose/bbox frame
  delivery remains independent;
- focused backend and launcher tests cover proxy replacement, visible load and
  render failures, and the remote dependency preflight.

Focused validation after this change: 40 tests passed across compositor,
compositor-process, SAM-launcher, and supervisor coverage.  No hosts were
launched and the alien3 environment has not yet been modified.

### Next task and acceptance criteria

Finish and validate only the remote mesh activation/render path:

1. install/provide `pytorch3d` in alien3's `coop3r-slam` environment and make
   the existing preflight pass;
2. run a real-GLB CUDA smoke test in that exact environment before replay;
3. repeat the concurrent 10-Hz MCAP launch and demonstrate ordered 501-frame
   rendering with zero gaps and at least the two ready meshes visible in
   post-arrival frames.

The earlier successful mesh-overlay artifact reports roughly 260--740 ms for
some raw-mesh frames.  After dependency repair, measure render latency rather
than assuming the 0.4--0.6-million-face assets sustain 10 FPS.  If they do not,
optimize an immutable cached render representation (for example, one-time
decimation at activation) without dropping or reordering authoritative
pose/bbox snapshots.

Do not change the durable pose/bbox stream, direct-MCAP ordering, or the
resident inference policy unless the instrumented failure demonstrates a
dependency on one of them.
