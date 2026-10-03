# CooperScene reference refinement: findings and limits

## Conclusion and correction to the previous report

**The earlier “corrected GT” interpretation is withdrawn.** The archive quaternion was already preprocessed; it was not raw INS output. The released chain transforms both position and attitude consistently, then applies intentional geometric ICP refinement. Removing those refinements and treating the metadata quaternion as raw was not a justified reference repair.

Retain the released ICP-refined trajectory as one reference candidate. Compare it with independently decoded original INS axes and explicitly conditional sensor-calibration hypotheses. Neither is certified ground truth. A separate alignment between the released coordinate frame and the GSplat is necessary for meaningful map-relative evaluation, but one road-plane calibration did not resolve the remaining height errors.

Infrastructure point clouds were used **only for offline debugging and provenance**. The tested map-tie correction and exported reference profiles require vehicle data, recorded calibration and the GSplat, not infrastructure PCD availability.

## What was established about the release

Here `E` is the stored Xsens-to-LiDAR calibration matrix, `C` is the per-take/per-agent ICP correction, and `W` is the supplied ENU-origin map tie. The actual numerical chain is:

```text
S_raw = Pose(original_INS_rotation, original_GNSS_ENU_position)
F_agent1 = Rx(180 degrees); F_agent2 = F_agent3 = identity
metadata_pose = E_agent @ S_raw @ F_agent
released_pose = C_take_agent @ metadata_pose
map_pose_candidate = W @ released_pose
```

- All 15 stored `C` matrices were reproduced from the original ICP inputs with maximum matrix-entry difference **5.49e-13**. The algorithm fixes infrastructure agent 0, picks the frame with minimum summed pairwise agent distance, and progressively aligns/fuses the vehicles. One correction is reused throughout each take; there is no clock-offset estimation or scan deskew.
- **299 timestamp-matched samples** from original sensor CSVs reproduce metadata attitude preprocessing within **2.01e-10**. Sixteen cropped-endpoint samples were excluded from the exact-match statistic. This supersedes the earlier claim that attitude was untouched.
- The converter agrees with the native convention. No Euler-sign change is warranted.
- Historical calibration code generates `E` from relative MoCap rigid-body poses. Exact marker-to-sensor/device conventions and the original calibration CSV remain unavailable. Thus using `E` as a physical mounting extrinsic remains a hypothesis, even after correctly undoing metadata attitude preprocessing.

See [release provenance and composition details](../../artifacts/reference-refinement/release-audit/FINDINGS.md). The conditional normalized-sensor helper was checked against all 6,705 newly exported poses. Old artifacts are preserved, not silently replaced.

## Identify the methods in the previous gallery

| Previous case | Estimated-localization row |
|---|---|
| Take 1 / agent 3, frame 464 | BEV + height |
| Take 2 / agent 3, frame 871 | BEV + height |
| Take 3 / agent 1, frame 0 | BEV + height |
| Take 3 / agent 3, frame 500 | BEV + height |
| Take 5 / agent 1, frame 180 | Two separate panels: XYZ and BEV + height |
| Take 5 / agent 3, frame 71 | BEV + height |

The new gallery shows **both existing localization methods on each identical frame**, alongside the reference candidates. Rejected saved estimates are explicitly marked; their visually good plane alignment does not override registration rejection.

## Same-frame road measurements

Each cell below is **normal-angle disagreement in degrees / signed road-height separation in meters**. Positive height means the placed LiDAR road is above the map road at the same map XY. These are road geometry diagnostics, **not localization position error or ATE**.

The six unique previous-gallery frames are unchanged. Two labelled controls are added: take 4 / agent 2 frame 134 and take 1 / agent 1 frame 0. No frame was selected to improve this comparison. The same LiDAR road points are transformed for every variant within each case.

| Take/agent, frame | Released ICP + W | Normalized INS + conditional body calibration | Released + fixed map tie | Existing XYZ | Existing BEV + height |
|---|---:|---:|---:|---:|---:|
| 1/3, 464 | 4.43° / +0.468 | 3.19° / +0.519 | 2.04° / -0.491 | 1.46° / +0.312 | 1.46° / -0.026 |
| 2/3, 871 | 4.08° / +0.443 | 2.16° / +0.514 | 1.52° / -0.450 | 0.89° / +0.291 | 0.89° / -0.111 |
| 3/1, 0 | 2.47° / +1.156 | 2.79° / -0.397 | 0.31° / -0.234 | 0.44° / +0.746 † | 0.44° / -0.081 |
| 3/3, 500 | 2.53° / +1.774 | 1.38° / +0.591 | 0.65° / -0.473 | 0.27° / +0.799 † | 0.27° / -0.247 |
| 5/1, 180 | 2.97° / +0.793 | 1.98° / +0.279 | 0.50° / +0.297 | 0.57° / +0.259 | 0.57° / +0.147 |
| 5/3, 71 | 2.71° / +1.616 | 2.08° / +0.424 | 1.62° / +0.513 | 0.23° / +0.774 † | 0.23° / +0.036 |
| 4/2, 134 | 3.55° / -0.235 | 1.33° / +0.205 | 0.85° / -0.293 | 0.59° / -0.162 | 0.59° / +0.267 † |
| 1/1, 0 | 4.19° / -0.356 | 3.45° / +0.222 | 1.59° / -0.516 | 0.42° / -0.219 | 0.42° / +0.101 |

† Saved localization failed its original registration acceptance. No gate is relaxed and no pose is promoted because a road plane looks good.

All eight map/source plane pairs pass the reported common-location hull and footprint support checks. Map-road candidates use the documented broad map Z band **[-4.3, -1.3] m**, radius 15 m, and existing plane-fit configuration; the band is a semantic diagnostic assumption, not a supplied GT height. Plane normals are fitted, not forced to gravity. Hulls can bridge gaps, local roads can curve, and Gaussian centers need not describe a single layer perfectly. Full residual/support details are in [road-comparison.csv](../../artifacts/reference-refinement/road-comparison.csv) and each case’s JSON/NPZ.

## Infrastructure debugging evidence

At the same supported road location, infrastructure and GSplat road normals disagree by **2.36–2.78°**, and infrastructure road height is **0.259–0.326 m above** the GSplat. This static mismatch cannot be caused by moving-vehicle timestamp skew. It demonstrates that aligning vehicles to infrastructure does not automatically align them to this GSplat.

The infrastructure’s `raw_gnss.csv` is byte-identical to agent 1’s in all five takes. Its fixed pose has no independently verified survey provenance in the available files. Infrastructure geometry is independent of GSplat localization, but is **not independent validation of the released ICP**, which was fitted to it. It is therefore useful diagnostic evidence, not absolute GT.

See [infrastructure audit](../../artifacts/reference-refinement/infrastructure-audit/report.md). Its older physical-INS comparison is explicitly marked superseded; the infrastructure planes and release-versus-pre-ICP evidence remain valid.

## Vehicle-only fixed road/map calibration experiment

One calibration frame was declared before testing: **take 1 / agent 1, frame 433 (native ID 481693)**, the original ICP anchor, excluded from the eight comparison frames. Only its vehicle LiDAR and GSplat road planes were used. One transform was frozen: roll **−0.586°**, pitch **+2.661°**, yaw **0°**, translation **[0, 0, −1.719] m**, about the fixed map origin. This is a coordinate-frame correction, not a per-frame height adjustment or the road height itself.

| Held-out metric, median across 8 frames | Released reference | Fixed map tie | Frames improved |
|---|---:|---:|---:|
| Road normal disagreement | 3.258° | 1.182° | 8/8 |
| Absolute road-height separation | 0.631 m | 0.461 m | 4/8 |
| Full static-cloud NN median residual | 0.363 m | 0.294 m | 6/8 |
| Full static-cloud NN p95 residual | 1.106 m | 0.701 m | 8/8 |

This is evidence for a shared orientation mismatch, **not a complete GT correction**. Four height errors and two full-cloud median residuals worsened. The frozen correction is map-calibrated and must not be called independent truth. See [calibration and held-out details](../../artifacts/reference-refinement/vehicle-road-calibration/report.md).

A separate source-only hypothesis preserves the original ICP anchor while changing sensor relative motion (`K = C B(anchor) inverse(P(anchor))`). Even with corrected metadata normalization it does not consistently improve the eight road comparisons. It remains an unpromoted diagnostic; do not simply reuse numeric C on differently composed sensor poses.

## Timing, slope and remaining disagreement

For these eight frames, LiDAR–GNSS skew is **4.85–73.67 ms**, corresponding to **0.044–0.804 m** first-order motion. Timing can substantially affect horizontal alignment. However, the largest modeled vertical-plus-road-slope translation contribution is only **0.0197 m**, much smaller than the released road-height gaps of **0.235–1.774 m**. Translation timing alone therefore cannot explain those gaps. Angular timing, smoothing, lever-arm effects and scan deskew remain unbounded. See [timing calculations](../../artifacts/reference-refinement/timing-audit.md).

The remaining discrepancy cannot yet be assigned uniquely among local road geometry, infrastructure/map frame calibration, marker-to-sensor axes/origins, vehicle attitude bias, GNSS elevation/datum, and timing. Better full-cloud ICP alignment can compensate these effects; it does not establish an independent reference. A plane constrains tilt and one height relation but does not independently resolve in-plane XY/yaw.

## Recommendation

1. Keep the **released refined reference** and the correctly decoded **conditional INS/sensor reference** distinct. Do not promote the previous metadata-based “physical correction.”
2. Treat the frozen vehicle-only map tie as a useful orientation diagnostic, not a new universal reference: residual height errors are still too large and inconsistent.
3. For a defensible map-relative benchmark, establish one map-to-release calibration from designated, geometrically diverse **vehicle** calibration frames, and test it on held-out vehicles/takes. Use static non-road structures as well as road surfaces; do not fit each evaluation frame to manufacture GT agreement.
4. Validate device/marker axes, GNSS reference point/altitude datum and capture-time poses before interpreting decimeter errors as physical accuracy. Propagate the verified frame consistently into priors and evaluation references.
5. Until those checks pass, report **registration consistency and reference disagreement**, not ground-truth accuracy, and do not re-rank XYZ versus BEV + height using the earlier <0.1 m counts.

No full teleoperation batch was launched. Existing queues, original results, acceptance gates and mounted source files were preserved.
