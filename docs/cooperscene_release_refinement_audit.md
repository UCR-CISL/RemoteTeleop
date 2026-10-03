# CooperScene release refinement and original INS audit

## Corrected finding

**The released metadata attitude is already calibrated.** Comparing it with the
original `raw_gnss.csv` disproves the earlier assumption that its quaternion was
raw INS output. The complete observed release chain is:

```text
S_raw(t) = Pose(R_original_INS(t), p_original_GNSS_ENU(t))
F_1 = Rx(180 degrees); F_2 = F_3 = identity
B(t) = E_a @ S_raw(t) @ F_a
N(t) = C_take_agent @ B(t)
```

Here `E_a` is the matrix stored under `towerN-xsens,towerN-lidar`; `F` has zero
translation. Both position **and attitude** were left transformed by `E`.
The earlier claim that only world position was rotated while attitude stayed
unchanged is retracted. Consequently the earlier `physical-references.json`
used a preprocessed quaternion as if it were raw and must not be labeled a
physical correction or ground truth.

The original ICP refinement is intentional: all 15 stored corrections for takes
1–5, agents 1–3 reproduce from original clouds and metadata to **5.49e-13** maximum
matrix element difference. Removing it discards actual geometric refinement.

## Evidence and generation chain

1. Original Xsens `.txt` headers identify the ENU output convention and enabled
   orientation/position smoothing. Original release `raw_gnss.csv` retains the
   original quaternion separately from the changed metadata quaternion.
2. Comparing 21 frames per sequence yields 315 comparisons. Of these, **299**
   match original GNSS timestamps within 128 ns and reproduce
   `R_metadata = E.rotation @ R_original_INS @ F_a` within **2.01e-10** maximum
   matrix element error. Sixteen cropped raw-CSV endpoint samples are missing
   and their nearest records differ by 2.5 ms; they are excluded from the exact
   identity statistic. `raw-attitude-audit.json` records every comparison.
3. The earlier position audit independently establishes
   `stored_enu = E.rotation @ raw_GNSS_ENU + E.translation` to numerical precision.
   Together these identities establish a full rigid left composition.
4. The ICP generator constructs `B` directly from stored ENU and metadata
   quaternion, selects one frame minimizing all pairwise agent distances, and
   fixes infrastructure agent 0. It refines the closest vehicle by inverse
   infrastructure-to-vehicle ICP, then other vehicles against the progressively
   fused aligned cloud in infrastructure-distance order. Parameters are
   point-to-point ICP, identity initialization, 3 m correspondence radius,
   maximum 200 iterations. Its `fitness_threshold` argument is unused.
5. It repeats one constant correction matrix for each vehicle throughout a take.
   This is not a timing, scan-deskew, or trajectory-optimization correction.
6. SensorPlatform `659476d` (October 4, 2025) explicitly fixes the MCAP writer from
   `B @ inverse(C)` to `C @ B`; the later `1144b95` label generator uses the
   consistent inverse order `inverse(B) @ inverse(C)`. Local `origin/main`
   (`1ad727f`) predates the writer fix. Its ICP code reproduces the stored values;
   its README describes an older algorithm and is not exact execution evidence.
7. Native YAML, converted matrices, MCAP/cache poses and local point coordinates
   agree in the earlier audit. No Euler sign/transpose fix is indicated.

| Take | Actual fitting frame | Vehicle fusion order | Largest C element discrepancy |
|---|---:|---|---:|
| 1 | 481693 | 1, 3, 2 | 3.77e-13 |
| 2 | 488685 | 2, 1, 3 | 2.44e-13 |
| 3 | 497206 | 2, 1, 3 | 3.05e-13 |
| 4 | 515587 | 1, 2, 3 | 4.79e-13 |
| 5 | 524102 | 2, 3, 1 | 5.49e-13 |

These selected frames are also the minima over all retained metadata, including
frames whose point clouds were later cropped. Each stored correction file has
one unique matrix. No evaluation-map geometry was used for reproduction.

## What the calibration source establishes

MultiModalCalibration commit `4e2e877007a4be28e717744ed317d2e4cd5c3e49`, October 1,
2025, is titled “cooperscenes mocap data processing.” Its
`scripts/extract_or_read.py` writes exactly the two pickle keys used here:
`(towerN-xsens,towerN-lidar)` and `(towerN-lidar,towerN-camera)`. It extracts row 0
from a MoCap CSV and calls `calculate_transform_F1_to_F2`; the implementation is:

```text
E = inverse(MoCap_T_target_rigid_body) @ MoCap_T_source_rigid_body
```

That generator produces a relative MoCap rigid-body transform, not a fitted
GNSS-world alignment. It does not apply additional marker-to-sensor calibration.
The exact original MoCap CSV has not been found, so its numerical output cannot
yet be reproduced as the ICP matrices were. The pickle keys, source timing and
camera-extrinsic agreement support this generator's relevance but do not prove
marker axes/origins coincide with the Xsens reported pose or physical LiDAR.
Nor is the original Xsens alignment/antenna-lever-arm configuration available.

Therefore the physically composed alternative remains **conditional** on those
sensor/marker conventions. A definitive claim that the release's left use of
`E` is a mounting-calibration bug would require those conventions. Regardless,
it is false to say that release preprocessing transformed position only.

## Reference candidates and what to retain

Use active column-vector transforms and let `W` be the existing `map_T_ENU`.

**Released refined reference:** retain `W @ N(t)`. This preserves the geometric
refinement anchored to infrastructure and requires no map fitting. Infrastructure
survey provenance and its exact relation to the map remain uncertain. Its
upstream history does not itself invalidate this released reference.

**Normalized original INS axes:** remove only the proven metadata left rotation:

```text
R_normalized(t) = E.rotation.T @ R_metadata(t) = R_original_INS(t) @ F_a
S_normalized(t) = Pose(R_normalized(t), p_original_GNSS_ENU(t))
axis_baseline(t) = W @ S_normalized(t)
```

This is a normalized INS-axis pose, not independently established LiDAR ground
truth. It is exported separately to avoid assuming a mounting calibration.

**Conditional mounting-calibrated baseline:** if the MoCap Xsens and LiDAR frame
conventions correspond to the normalized INS and physical LiDAR, use:

```text
P(t) = S_normalized(t) @ inverse(E)
sensor_baseline(t) = W @ P(t)
```

This removes the stored ICP correction and is a conditional raw sensor baseline.
Altitude datum, GNSS origin, marker alignment and capture-time qualifications
remain. It is not established to improve geometry.

**Preserve the proven refinement anchor while changing relative motion:**

```text
K_a = C_a @ B_a(t0) @ inverse(P_a(t0))
anchor_candidate(t) = W @ K_a @ P_a(t)
```

Here `t0` is the actual fitting frame reproduced above. This exactly preserves
release alignment at `t0`, while applying the conditional sensor relative motion
elsewhere. It requires heldout validation and inherits infrastructure-anchor
uncertainty. It is not a proven improvement. At use time only vehicle metadata
and saved profiles are required; infrastructure PCDs were used for offline
provenance/debugging and are not a deployment dependency.

**Change a common world frame:** for independently established
`D = new_world_T_release_world`, transform complete poses as `D @ N`. If writing
corrections in that new coordinate basis, use `C_new = D @ C @ inverse(D)` and
`B_new = D @ B`. Reusing numeric `C` unchanged on the conditional `P` is not a
valid repair: ICP was estimated after `E` preprocessing and absorbs its effects.
Separating environmental refinement from calibration compensation is generally
not possible from `C` alone. New capture-time alignment, sensor calibration, or
infrastructure calibration warrants re-estimating/refining registration and
validating heldout frames.

## Artifacts and reproduction

All files below are under `artifacts/reference-refinement/release-audit/`:

- `release-audit.json`: exact ICP reproduction, source revisions and SHA-256s.
- `raw-attitude-audit.json`: original INS versus metadata comparisons and MoCap
  generator source provenance; source snapshots are retained alongside it.
- `raw-normalized-references.json`: 6,705 conditional mounting-calibrated poses.
- `normalized-ins-axis-references.json`: 6,705 normalized original INS-axis poses.
- `raw-normalized-anchor-references.json`: 6,705 conditional poses preserving the
  actual ICP fitting anchor, with exact frame keys and `map_T_dataset` fields.
- `raw-normalized-anchor-profiles.json`: compact per-sequence profiles. These
  require the proven metadata attitude undo before applying the stored `W @ K`.

Earlier `physical-references.json` and `anchor-repaired-references.json` remain
unchanged for auditability. Their physical interpretation is superseded because
they treated the already-calibrated metadata quaternion as raw INS.

```sh
OMP_NUM_THREADS=4 OPENBLAS_NUM_THREADS=4 \
/home/jyue86/Documents/CISL-Projects/RemoteTeleop/.venv/bin/python \
  scripts/investigation/audit_cooperscene_release.py --reproduce-icp
/home/jyue86/Documents/CISL-Projects/RemoteTeleop/.venv/bin/python \
  scripts/investigation/audit_cooperscene_raw_attitude.py
/home/jyue86/Documents/CISL-Projects/RemoteTeleop/.venv/bin/python \
  scripts/investigation/export_cooperscene_raw_normalized_reference.py
```

These commands read source data and write new audit artifacts only. No mounted
release files, prior worktrees, vehicle processes, evaluation results or running
queues are modified. Original execution commands/runtime and the source writer
of `E @ S_raw @ F` remain unavailable; the numerical identities above establish
its actual operation independently.

## Validation of the archive-aware implementation

The explicit `CooperSceneReference.map_T_lidar_from_release_metadata` method
agrees with every one of the 6,705 independently exported normalized sensor
poses to 1.42e-14, and all corresponding anchor poses to 5.69e-14. The exporter
now uses this method. It copies metadata before undoing the attitude transform.
`raw-normalized-validation.json` records this check and eight fixed road-plane
comparisons using the same source and map planes as the parent audit.

The conditional anchor-motion candidate does not show consistent improvement
on those eight cases. For example, take 5 / agent 1 / offset 180 has a
1.98-degree normal difference and +0.279 m road-height separation for the
normalized sensor hypothesis, versus 2.85 degrees and -1.677 m for the anchor
hypothesis. These are local geometric diagnostics, not certified pose errors.
The exact fitting-frame identity alone does not justify promoting the anchor
trajectory. Retain the released refined baseline and compare the explicitly
conditional sensor hypothesis while independent calibration remains unresolved.
