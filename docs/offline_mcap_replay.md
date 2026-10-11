# Offline MCAP replay and asset rendering

The replay queue uses recorded MCAP inputs, procedural vehicle assets and the
existing remote compositor. It supports CooperScene agents 1–3 and nuScenes,
selected start-frame offsets, isolated attempts, retries, and resume. Input
readiness checks verify a completed MCAP footer/summary, camera counts and map
files. No SAM3/SAM3D workers are needed in asset-proxy mode.

```sh
.venv/bin/python scripts/deployment/run_remote_teleop_batch.py \
  --manifest path/to/manifest.json --output-root artifacts/replay-NEW --dry-run
```

Remove `--dry-run` only when intentionally launching data playback on prepared
hosts. `prepare_remote_teleop_batch.py` discovers and validates MCAP/map inputs;
`stage_remote_teleop_batch.py` stages isolated code/dependencies and has its own
`--dry-run`. Neither is automatically run by importing these modules.

Each nuScenes manifest entry must name its scene-specific map. CooperScene entries
must name the intended Riverside map and explicit cached localization transform.
The manifest preparer's recorded-pose reference defaults are historical; they do
not automatically apply the new geodetic calibration recipe. The retained recipe
operates on raw-GNSS/INS poses and must not be applied directly to released
ICP-refined coordinates. Localization metrics label the supplied reference.

Vehicle meshes and wireframe fallbacks share one object depth buffer as well as
Gaussian scene-depth tests. Near assets occlude farther assets regardless of
iteration order. Render grounding is explicit: `--ground-plane` accepts a plane
or bounded road-surface JSON. `--ground-z` on the compositor accepts a fixed map Z;
it is mutually exclusive with `--ground-plane`. Omitted grounding preserves the
localized object height. Ground placement modifies rendered boxes, not localized
sensor poses or trajectory-error measurements. Unsupported bounded ground
queries preserve the incoming pose and emit per-track placement diagnostics.

Validation used simulated launchers/readers and local tests only; no replay or
real vehicle process was launched during integration.

For the later deterministic detector/tracker evaluation on Lambda and Gamma,
see [CooperScene tracking pilot](cooperscene_tracking_pilot.md). It uses native
capture timestamps, the retained raw-GNSS/INS XYZ policy, recorded detector
outputs, and explicit front-camera coverage checks. It does not launch vehicle
control or change the localization policy of older MCAP replay manifests.
