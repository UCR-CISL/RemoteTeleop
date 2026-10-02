# Hybrid road-height pilot

Recommendation: retain as experimental; no new replay batch or unchanged all-23 expansion.

Compared existing frozen BEV/XYZ replay transforms and hybrid on 18 identical,
predeclared frames: nuScenes 152 and CooperScene 1/1, 2/2, 1/3, 3/2, 4/3.
Frames are first common replay frame, midpoint and last, not newly selected starts.

| Outcome | Frames |
|---|---:|
| LiDAR local road fit available | 18 |
| Ambiguous map layers | 6 |
| Unresolved map-layer search | 5 |
| Insufficient compatible map support | 3 |
| Height candidates | 4 |
| Passed original registration gates | 1 |

The sole accepted correction, nuScenes 152 frame 190, leaves horizontal reference
error at 0.1855 m but changes Z error from approximately zero to −0.1027 m;
XYZ error increases to 0.2121 m. Its BEV height was reference-conditioned, so this
is not evidence of independent ground-truth error. No CooperScene height
correction passed the unchanged registration gates. Strong LiDAR plane fits do
not resolve ambiguous GSplat ground or source/map tilt.

Full extrinsics transport LiDAR road clearance to the camera. Corrections affect
sensor/camera poses only and preserve frozen BEV transverse translation and
rotation. Because BEV up differs from reference up, rejected CooperScene
proposals move reference XY by 5–13 cm. Rendering-only grounding is absent.

The height estimator receives no target/reference height. Existing BEV
initialization, earlier frame selection and ENU fusion remain reference-conditioned;
dynamic masks use recorded annotations. Predictions are saved before evaluation.
Original dataset registration masks and acceptance gates remain unchanged.

Full tables, signed XYZ/RPY, residual/support metrics, audit and camera/side-view
visuals are preserved locally:

- [Report](/tmp/RemoteTeleop-hybrid-road-height/artifacts/hybrid-road-height/pilot-v3/report.md)
- [Metrics CSV](/tmp/RemoteTeleop-hybrid-road-height/artifacts/hybrid-road-height/pilot-v3/metrics.csv)
- [Visuals](/tmp/RemoteTeleop-hybrid-road-height/artifacts/hybrid-road-height/pilot-v3/visuals/index.md)

Run `scripts/investigation/evaluate_hybrid_road_height.py --output <new-directory>`
with the existing local data caches; the committed pilot JSON freezes frames.
Render with `python -m src.viz.diagnose_hybrid_height --results <new-directory>/results
--output <new-directory>/visuals`. Use the repository environment with gsplat/CUDA
for camera images. No remote queue changes or mounted-source writes are needed.

Verification: 25 targeted tests passed, covering full extrinsics, fixed axes,
ambiguity/fallback, reference-invariant acceptance and unchanged registration gates.
Next address reference-free map-road identity/support and independently audit tilt
and capture timing before repeating this exact pilot.

## Follow-up: reusable clearance under a metric, accurate-road map assumption

The simpler hypothesis is supported by 38 LiDAR-only road fits across all 23
first frames and 15 additional pilot frames. Fixed calibration used nuScenes
152 frame 0 and CooperScene take 1 frame 0 separately per vehicle.

| Held-out measurements | Within 0.1 m of fixed LiDAR clearance |
|---|---:|
| Other nuScenes first frames | 6/7 |
| CooperScene takes 2–5, same vehicle | 11/12 |
| Additional pilot frames | 15/15 |

CooperScene per-agent nominal LiDAR clearances are 1.834, 2.220 and 1.979 m;
a single dataset-wide mounting height is inappropriate. Outliers are nuScenes
164 and CooperScene 3/2, approximately 0.12–0.13 m from calibration.

The known nuScenes first-camera map anchor allows each initial LiDAR road plane
to be transformed directly into map coordinates without GSplat matching. This
works for all eight scenes under the stated export convention. That resolves
initial road placement, not independent first-frame pose accuracy. In scene 152,
measured vertical clearance varies only 2.52 cm, while extrapolating the first
road plane disagrees by 13.6/17.0/34.7 cm at frames 36/95/190 using recorded
relative poses for diagnostic evaluation.

The separate conditional Riverside Z=-2.8 experiment does not validate that
rendering plane for localization: 2/15 measured-clearance and 1/15 transferred-
clearance proposals pass unchanged registration gates; accepted changes worsen
reference agreement. Clearance consistency and absolute map elevation remain
separate claims. Existing queue and source data were untouched.

[Follow-up report, all-sequence tables and plots](/tmp/RemoteTeleop-hybrid-road-height/artifacts/clearance-transfer/report.md).
Reproduce with `scripts/investigation/evaluate_clearance_transfer.py`,
`audit_camera_anchored_road.py`, `evaluate_fixed_ground_clearance.py`, and
`python -m src.viz.plot_clearance_transfer` in the repository environment.
