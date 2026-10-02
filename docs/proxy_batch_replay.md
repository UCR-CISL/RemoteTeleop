# Lambda / Alien3 proxy replay queue

The batch launcher runs dataset playback on Lambda and the GSplat compositor on
Alien3, one run at a time. It always enables procedural vehicle assets. Bounding boxes default to recorded
annotations; each run can select `bbox_source: bevfusion` for inference on Lambda. No SAM3 or SAM3D workers are started.

The default preparation covers eight available nuScenes scenes (152, 164, 171,
200, 209, 359, 529, 916) and CooperScene takes 1–5, agents 1–3: **23 runs / 8,263
frames**. Agent 0 has no front camera. CooperScene files are read from the mounted
source directory and copied once per take; the source directory is never changed.

## Stage, prepare, run

Run from this worktree with a Python environment containing this project's base
dependencies. Both hosts must be reachable over SSH. `uv` must be available on
Lambda for the isolated protobuf dependency installation.

```bash
python scripts/deployment/stage_remote_teleop_batch.py
python scripts/deployment/prepare_remote_teleop_batch.py \
  --cooperscene-localization-transform \
  /home/jyue86/Documents/CISL-Projects/RemoteTeleop/artifacts/cooperscene_fixed_xyz_final/localization.json
python scripts/deployment/run_remote_teleop_batch.py \
  --manifest artifacts/proxy-batch/manifest.json \
  --output-root artifacts/proxy-batch/results --wait-for-inputs
```

Use a durable terminal such as tmux for the queue. Each command accepts `--dry-run`;
dry runs print the plan without connecting, transferring data, or starting services.
Preparation needs live MCAP inspection to produce a real manifest, so its dry run
does not create one. Add `--continue-on-error` to the final command to proceed to
later runs after a failed run. The default stops at the first failure.

Staging places code in separate `RemoteTeleop-proxy-batch` directories on both
hosts. It uses the existing Python/CUDA environments through `batch-python`
wrappers; supplemental protobuf packages are isolated under `.batch-site` on
Lambda. Existing checkout files and environments are not overwritten. The
latest procedural car and loader geometry is included in this worktree.

Preparation copies the nuScenes Gaussian maps from Lambda through a local cache
to Alien3, and the five CooperScene MCAPs to Lambda. Rsync uses partial transfers
and does not delete destination files. It reads each nuScenes recording's own
camera calibration and actual frame count. It checks required files and writes
the manifest only after successful preparation. The five CooperScene recordings
total approximately 31.9 GiB; allow this space on Lambda in addition to run
outputs. The local map cache requires approximately 2.1 GiB.

CooperScene uses the current `riverside_iowa.ply` on Alien3 and the matching
`cooperscene_fixed_xyz_final` alignment. Preparation requires XYZ mode and a
matching map filename. NuScenes requires an accepted per-scene full 3D registration
under `artifacts/batch-localization/nuscenes/<scene>/` and pairs each scene with
its own `background_40000.ply` export.

Every configured run logs `lambda/localization-trajectory.jsonl` and
`lambda/localization-trajectory.summary.json`. The launcher requires exactly one
metric per frame and a complete XYZ summary before declaring success. Rows include
signed dx/dy/dz, 3D distance, horizontal and vertical distance, orientation error,
and both poses. Signed components use dataset axes (Z up), even when the Gaussian
map itself uses camera-oriented axes. Summaries contain mean, RMSE, p95 and max.

CooperScene errors are relative to the shared ENU prior; nuScenes errors compare
the registered trajectory against its recorded-pose map alignment baseline.
These are explicitly labeled reference disagreements, not independent ground-truth
accuracy. No post-hoc trajectory alignment is fitted to reduce the reported error.
The fixed localization transform is propagated to every recorded sensor pose.

## State and outputs

The local queue root contains `state.json`, with queued, running, succeeded and
failed status plus attempt history and error messages. Each attempt gets its
own local and remote output directories. A successful run has passed the existing
launcher's frame-count and completion-marker checks and has a local video path
in state. Rerunning the same command skips successes and retries failures with
fresh attempt directories. A changed manifest requires a new queue root.

The queue root is exclusively locked while executing. Use one queue for this
host pair: separate queues would still compete for the same network ports/GPU.
Normal exceptions and Ctrl-C invoke the existing launcher cleanup. After a forced
process kill or host failure, inspect and stop any remaining remote services
before resuming an interrupted run with `--retry-interrupted`; the harness
otherwise refuses stale running entries. A new output directory does not free
old ports.

## Monitored launch

`--max-runs 3` limits a launch to three pending entries, leaving the remaining
entries queued. Resume with the same manifest and queue root after reviewing the
first runs; successful entries are skipped. Do not edit the manifest of an active
queue. Dataset transfers must finish before their runs are released.

With `--wait-for-inputs`, queued entries poll for a complete MCAP footer, expected
camera/box counts, transform files, and the scene map. No attempt is consumed while
waiting. The default input timeout is six hours (`--input-timeout-seconds`).

The monitored October 1 batch uses `artifacts/proxy-batch/queue-manifest.json` and
`artifacts/proxy-batch/xyz-results/state.json`. First-three reports and previews
are under `artifacts/proxy-batch/monitor/`. The remaining queue runs in tmux session
`proxy-batch-remaining` with log `artifacts/proxy-batch/remaining.log`.

## Localization comparison queues

`prepare_localization_evaluation.py` builds a separate manifest for each evaluation.
Set 1 uses the best accepted XYZ fit observed in the prior baseline frame search
within the first 25% of each recording, with BEVFusion detections. Set 2 uses the
best accepted XY fit from the exhaustive BEV search in that window, recorded boxes,
and audited map-space ground planes. Selection uses the comparison reference and
is therefore a retrospective evaluation, not an online localization policy.

The current comparison uses **BEVFusion detections for both localization modes**.
XYZ has no ground placement; its original set 1 manifest already follows this
policy. The earlier grounded XYZ scene-152 diagnostic is not the selected policy.
Set 3 (`set3-bev-bevfusion-grounded/manifest.json`) replaces recorded boxes in
the BEV evaluation with actual BEVFusion inference, retaining the same selected
frames and localization transforms. It uses bounded local road support for
nuScenes and the fixed Riverside Z = -2.8 m plane for CooperScene. The shared
vehicle depth buffer applies independently of ground placement.

A fit must have error below 0.1 m to select its frame. Otherwise playback starts at
frame zero with the original inherited transform, explicitly marked as a fallback.
`mcap_start_sequence` selects the original source offset; `expected_samples` counts
remaining frames. Output sequence/timestamps are rebased while source provenance is
retained. `localization_position_mode` selects XYZ or XY summaries; all XYZ errors
remain logged along the remaining trajectory. `ground_plane` names an audited JSON
plane on the renderer host. CooperScene BEVFusion uses a nuScenes-trained detector,
so this is a cross-dataset detector evaluation.

The current evaluation artifacts live under `artifacts/localization-evaluation/`.
`set1-xyz-bevfusion/manifest.json` runs in tmux `localization-eval-set1`. The durable
`localization-eval-set2` waiter requires set 1 termination, complete BEV search, and
an audited/staged plane index covering all 23 runs before preparing and launching
`set2-bev-grounded/manifest.json`. Logs and exit markers are `set1.log`, `set1.exit`,
`set2.log`, and `set2.exit`; `first-smoke.json` captures the initial replay check.
The corrected BEVFusion BEV queue runs in tmux `localization-eval-set3`, with
`set3.log` and `set3.exit`. Previous evaluation outputs remain separate.
