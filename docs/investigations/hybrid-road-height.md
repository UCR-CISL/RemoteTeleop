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
