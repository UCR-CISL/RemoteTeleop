# Shared ENU calibration from maximum-overlap vehicle frames

## Protocol

Keep every released infrastructure and vehicle pose unchanged. Search all 6,705
original vehicle PCDs across takes 1–5. Select one frame per physical vehicle by
the largest overlap with stationary infrastructure, then fit **one** rigid
CooperScene-to-GSplat transform using those three full static clouds and one
stationary infrastructure scan. No individual vehicle correction is fitted.

Selection uses shared occupied 0.5 m voxels against one fixed, deduplicated
infrastructure target, built from the previous 15 infrastructure calibration
scans. Its geometry is independent of GSplat. Ties use Dice, then earlier
take/frame. Recorded vehicle boxes are removed with 0.2 m padding. The scoring
grid is fixed in the original-W translated ENU frame; overlap depends on
sampling/grid boundaries and is not a pose-accuracy score.

All 6,705 frames were scored; 6,667 were eligible. The 48 previously reserved
identities comprise 38 vehicle frames and 10 infrastructure frames; only the
38 vehicle exclusions affect this search. The original eight comparison frames
and all 40 held-out scans remain separate from calibration. Runtime was about
16.8 minutes reading the original archives, without modifying them.

## Selected frames

| Vehicle | Take | Native frame ID | Offset | Elapsed time | Sequence completed | Infrastructure voxel coverage |
|---|---:|---|---:|---:|---:|---:|
| 1 | 1 | 481677 | 417 | 41.700 s | 83.40% | 40.56% |
| 2 | 2 | 488685 | 835 | 83.497 s | 94.89% | 33.76% |
| 3 | 2 | 488684 | 834 | 83.401 s | 94.77% | 20.77% |

The fitted union includes one predeclared stationary scan: take 1 / unit 0 /
frame 481510, offset 250. The aggregated infrastructure is used for selection;
it is not repeatedly added to the primary fitted cloud. Full selected vehicle
clouds are retained, including their coverage outside the infrastructure view.

Fifteen per-take winners are also saved. One unrestricted per-take winner was a
reserved case (take 3 / vehicle 3, offset 500); its eligible replacement is
offset 499. This does not change the three global winners.

[All scores, timing and selections](../../artifacts/reference-refinement/overlap-selected-calibration/selection/report.md)
and [overlap throughout all sequences](../../artifacts/reference-refinement/overlap-selected-calibration/selection/overlap-through-sequences.png).

## Fused cloud and transform

The four-cloud union has 64,144 unique 0.25 m voxels. At the same fitted pose and
0.5 m occupancy grid, adding the selected vehicle clouds increases matched map
voxels from **1,629 to 17,481** (10.7×), or **0.926% to 9.935%** of occupied
map voxels. This is an exact voxel-intersection metric, not a physical surface
coverage percentage. Supported non-horizontal source points increase from
1,665 to 19,764. The proposed selection substantially adds complementary geometry.

The primary map proxy uses opacity >0.05. A positive-opacity control and two
infrastructure-only controls use the identical fitting method. The primary
correction relative to the original ENU tie has Euler XYZ rotation
`[-0.622767, 2.741349, 0.372694]` degrees and translation
`[0.176363, 0.507966, -1.613112]` meters. Column-vector composition:

```text
map_T_released_enu = D @ original_W
map_T_lidar = map_T_released_enu @ released_enu_T_lidar
```

[Exact matrix and provenance](../../scripts/investigation/cooperscene_overlap_enu_map.json).
The final fit has 39,680 supported correspondences and numerical rank six. It
reached its fixed iteration limit; the last update was 0.0000423° / 0.0000232 m.
That termination is retained explicitly, not relabelled as strict convergence.

## Separate-frame validation

| Identical validation inputs | Previous shared tie | Selected four-cloud tie |
|---|---:|---:|
| Median vehicle NN residual, 30 held-out scans, opacity >0.05 target | 0.335 m | 0.350 m |
| Median infrastructure NN residual, 10 held-out scans, same target | 0.190 m | 0.190 m |
| Median absolute road-height separation, same original eight planes | 0.328 m | 0.361 m |
| Median road-normal disagreement, same eight cases | 1.093° | 1.107° |

All eight road comparisons pass their original support checks. With a common
opacity >0.05 road surface instead, the eight-case median height is 0.414 m for
the previous tie and 0.446 m for the new tie. No map representation is changed
within a reported comparison. These are geometry residuals, not localization ATE.

The new same-method infrastructure-only control has a 0.336 m held-out vehicle
NN median, compared with 0.350 m for the union. This controls the additional
vehicle geometry. Comparison against the older shared fit also changes weighting:
the old fit balanced/capped individual groups; this fit uses unique fused voxels
and balances horizontal versus other normals across the union.

Fitting each take's selected triplet separately gives deviations from the primary
transform up to 1.327° and 0.789 m on the tested origin/±30 m cross. Combining all
15 selected vehicle scans gives a 0.652° / 0.512 m difference. These measurements
show source-selection sensitivity; they are not statistical uncertainty bounds.

## Interpretation

This experiment confirms the coverage benefit of combining the maximum-overlap
frames in their existing common ENU frame. It produces a fully recorded common
map transform. Held-out geometric consistency does not improve over the earlier
shared calibration, so this candidate does not establish a more accurate
evaluation reference or resolve the <0.1 m requirement.

Retain the candidate and its selected scans for inspection. Production defaults,
released poses, acceptance thresholds, previous results and evaluation queues
remain unchanged. No full replay batch was launched.

[Detailed fit, controls and stability](../../artifacts/reference-refinement/overlap-selected-calibration/fit/report.md)
and [combined top/side cloud views](../../artifacts/reference-refinement/overlap-selected-calibration/fit/combined-source-target.png).
