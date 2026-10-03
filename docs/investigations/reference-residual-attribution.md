# Remaining CooperScene reference error: attribution experiments

## Conclusion

The shared ENU-to-map calibration is useful, but no tested refinement establishes
a <0.1 m evaluation reference. Two separate effects remain: the map's measured
road surface depends on which Gaussian centers are included, and vehicle scans
do not consistently agree with infrastructure across takes. A further global
height shift or a fixed per-vehicle mounting correction does not resolve both.

Two Astra agents independently investigated vehicle/infrastructure and
infrastructure/map geometry. Parent review checked transform composition, local
road patches, source provenance, GNSS status, and support calculations. No
runtime setting, localization acceptance gate, mounted source, existing result,
or evaluation queue was changed. No full teleoperation batch was launched.

All distances below are **geometry consistency statistics, not sensor-position
error or ATE**. Infrastructure was the released ICP target and has no verified
survey provenance here; it is not independent ground truth.

## 1. Isolate map-surface selection from pose correction

Fit infrastructure alone using its 15 predeclared calibration scans, then check
10 held-out infrastructure scans and the unchanged vehicle frames. Preserve the
released poses and original train/held-out split. Gaussian opacity thresholds
are sensitivity experiments, not calibrated physical confidence.

| Frozen infrastructure pose; change only map surface | Positive-opacity centers | Opacity >0.05 |
|---|---:|---:|
| Median absolute infrastructure road separation, 37 supported patch/frame pairs | 0.163 m | 0.029 m |
| Maximum infrastructure road separation | 0.312 m | 0.138 m |
| Median infrastructure road-normal disagreement | 0.613° | 1.863° |
| Median absolute vehicle road separation, same eight frames | 0.378 m | 0.439 m |

The infrastructure result is largely a **surface-selection effect**: keeping the
original surface and changing only to the opacity-filtered fitted pose leaves
0.138 m median separation. It does not prove that filtering has recovered the
physical road everywhere: normal disagreement worsens and the maximum remains
above 0.1 m. The same filter does not fix vehicle road alignment.

The current CooperScene implementation filters opacity >0.05 in coarse
registration but uses opacity >0 for refinement
([implementation](../../src/localization/cooperscene_localization.py)). The shared
calibration and historical road diagnostics used the latter. Thus low-opacity
centers were not consistently excluded. No filter has been silently changed.

Infrastructure-only registration also does not provide a uniformly accurate
non-road reference. After correcting a tangent-basis bug in the diagnostic hull
test, 49 supported vertical patch/frame comparisons have 0.054 m median anchor
separation but **0.595 m maximum and 11.15° median normal disagreement**. Small
separation where two planes nearly cross must not be read as good wall alignment.
These are geometric patches, not verified semantic walls.

[Infrastructure/map report and plots](../../artifacts/reference-refinement/infra-map-residual/report.md)
and [same-eight surface controls](../../artifacts/reference-refinement/infra-map-residual/vehicle-surface-sensitivity.md).

## 2. Vehicle/reference disagreement persists with GSplat excluded

The second experiment compares 45 designated vehicle calibration scans and 30
held-out vehicle scans against per-take infrastructure aggregates. All 75
archived poses reproduce the released `C @ metadata_pose` chain to 9.95e-14
maximum matrix-entry difference. There is no new pose-conversion discrepancy.

Two correction models are kept distinct:

- **World correction:** `D @ released_pose`, fixed per vehicle.
- **Sensor correction:** `released_pose @ X`, a fixed mounting hypothesis.

Each held-out take is tested using corrections fitted to calibration scans from
the other four takes. Source support is frozen before comparing variants.

| Leave-one-take-out result | World correction | Sensor correction |
|---|---:|---:|
| Median common-support NN residual, 30 held-out scans; original 0.894 m | 1.053 m | 1.017 m |
| Scans whose NN median improves | 10/30 | 9/30 |
| Paired supported road comparisons | 12 | 14 |
| Median absolute road separation before → after, on those same pairs | 0.201 → 0.096 m | 0.206 → 0.149 m |
| Maximum absolute road separation before → after | 1.040 → 0.255 m | 1.040 → 1.808 m |

Road height sometimes improves while static/non-road consistency worsens. The
0.096 m median does not establish <0.1 m reference accuracy: the maximum is
0.255 m, many road pairs lack support, and other geometric evidence regresses.
Infrastructure sampling/occlusion contributes to NN residuals, so their absolute
magnitude is not pose error. Identical-point comparisons expose the tradeoff.

Vehicle 3 is particularly weakly constrained: only one of ten held-out road
comparisons is supported in the released placement, and its sensor correction
improves NN residual in zero of ten scans. This limits attribution; it does not
prove a particular physical mounting error.

Checking the original released ICP fitting frames separately yields supported
road comparisons for nine of fifteen anchors, with absolute height separation
0.014–0.202 m; six lack support. Even the original fitting anchors therefore do
not establish uniform <0.1 m alignment. Vehicle 3's fitted sensor correction
changes by up to 5.07° in roll and 1.097 m in a translation component when a take
is omitted. Algebraic full rank does not imply a stable physical calibration.

Heading spans approximately 181–183° across calibration observations, but
roll/pitch excitation is much smaller. The centered attitude-stack singular
values have a small third component (approximately 0.043–0.059), leaving vertical
lever-arm estimation weakly constrained. Neither left nor right correction
establishes the missing marker/device calibration provenance.

The infrastructure's sparse sampling required an explicitly documented
diagnostic normal-neighborhood revision. This is separate from existing
localization acceptance thresholds. Final results include planar support and
conditioning checks; unsupported fits must not be promoted as calibrated poses.

[Vehicle/infrastructure report, paired metrics and views](../../artifacts/reference-refinement/vehicle-infra-residual/report.md).

## 3. The remaining height gaps are spatially visible

The parent audit compares small 2.5 m road patches at identical XY locations,
requiring source and map support before and after the frozen shared calibration.
It uses the previously saved road inliers and therefore remains conditional on
that extraction; no evaluation pose is fitted.

| Case after shared calibration | Median signed local separation | Range across supported patches |
|---|---:|---:|
| Take 3 / vehicle 3 / frame 500 | −0.380 m | −0.498 to −0.233 m |
| Take 5 / vehicle 1 / frame 180 | +0.390 m | +0.241 to +0.448 m |
| Take 5 / vehicle 3 / frame 71 | +0.615 m | +0.319 to +1.040 m |

These are spatial ranges, not confidence intervals. A separate check using the
opacity >0.05 infrastructure fit **and** filtered map surface still leaves the
same three frames at −0.373, +0.373 and +0.631 m at their original comparison
anchors. Neither a single large-plane artifact nor low-opacity map centers alone
explains these cases.

[Local road-patch PNG](../../artifacts/reference-refinement/local-road-separation/local-road-separation.png)
and [measurements](../../artifacts/reference-refinement/local-road-separation/report.md).

## 4. Recorded GNSS status

Of 83 inspected vehicle records (75 manifest selections plus the previous eight
cases), 77 report RTK fixed and six report RTK float. All six float records are
take 3 / vehicle 1. The largest persistent error cases above report RTK fixed.
Matched original GNSS timestamps differ by at most 128 ns from metadata.

Decoding follows [Xsens low-level documentation, Table 27](https://www.xsens.com/hubfs/Downloads/Manuals/MT_Low-Level_Documentation.pdf).
Receiver status does not certify INS attitude, antenna lever arm, altitude datum,
sensor composition or map placement. It does not explain away the geometric
disagreement. [Full status audit](../../artifacts/reference-refinement/gnss-status-audit/report.md).

## Recommended direction

1. Retain the shared map transform as a named calibration candidate, not GT.
   Do not adopt either fixed per-vehicle correction or re-rank earlier <0.1 m
   localization counts from these results.
2. Make the map surface used for measurement explicit and consistent with the
   registration stage. The >0.05 filter is worth a controlled comparison, but
   it does not by itself establish physically accurate geometry.
3. Focus subsequent reference reconstruction on **verified static structure
   correspondences and capture-time poses within each take**. Road planes alone
   cannot constrain the full pose, and the tested broad NN fits do not supply
   reliable non-road constraints. Inspect the original ICP anchors and use
   geometrically supported calibration windows, with untouched held-out frames.
4. Preserve explicit insufficient-support cases. Recover the marker/device
   extrinsics and timing/lever-arm configuration where possible; fitting a
   correction that aligns one evaluation frame is not a substitute for those
   missing constraints.

The experiments narrow the causes and reject easy fixes. They do not yet
establish a defensible <0.1 m evaluation reference.
