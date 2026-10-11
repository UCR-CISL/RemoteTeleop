# Worktree import, 2026-10-11

Selected code was brought into the main `split-systems` checkout without staging
or committing. Sources were the reference-refinement branch and
`/tmp/RemoteTeleop-alignment-refinement`.

## Imported

- Retained recipe in `config/cooperscene_geodetic_alignment_recipe.json`.
- Raw GNSS/INS and sensor composition in `src/localization/cooperscene_reference.py`.
- Capture-time interpolation and recipe application consolidated in
  `src/localization/cooperscene_geodetic.py`, removing frame-search imports of
  older map-fitting/trajectory experiments.
- BEV/XYZ native-frame input, independent frame evaluation, provenance,
  concurrent queue and reports in `scripts/investigation/cooperscene_alignment/`.
  Default recipe lookup now uses the main repository config JSON.
- Bbox-adaptive vehicle meshes, supported labels, cached proxy rendering,
  preview generation, compositor integration and `--asset-proxy` launcher mode.
  The latest loader cabin correction is included. Proxy mode skips SAM3/SAM3D.

## Validation

225 relevant tests passed. Three-frame offline parity covered take 1/V1/frame
250, take 3/V1/frame 0, and take 4/V2/frame 150. Initial, raw registration,
BEV/XYZ fusion, rejection certificates and acceptance matched saved full-search
results within 1e-10, excluding runtimes. Evidence:
`artifacts/alignment-import-smoke-20261011/`.

The recipe also supplies initialization and the fusion prior, so initial error
is zero by construction. Evaluation measures reference agreement and map
consistency, not independent GT accuracy. The 3 cm antenna offset remains
unverified. Existing replay defaults have not been redirected to this recipe.

## Luna's review of other worktree changes

The remaining work consists of:

- **Remote batch replay:** SSH/rsync staging, resumable queue/state/locking,
  selected-frame replay, BEVFusion options, and trajectory metrics.
- **Rendering ground and occlusion:** shared object-depth buffering, bounded
  map-ground support, explicit Riverside height and render diagnostics.
- **Experimental localization:** BEV plus LiDAR height, hybrid road matching,
  height-transfer studies, infrastructure alignment and trajectory refinement.
- **Surface consistency research:** all-take surface evaluators/galleries,
  regression caches, and V1/V2/V3 residual fits. Recent candidates were not
  promoted because road improvements sometimes worsened fixed wall support.

Those groups remain in the investigation worktrees. The import uses the narrow
proxy integration rather than the later compositor/grounding/batch versions.
No teleoperation process was launched during import.
