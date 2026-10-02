# Full-sequence localization search

Current comparison entry point:
`scripts/investigation/search_full_sequence_localization.py`.
Historical limited-window scripts/results remain preserved for reproducibility;
this experiment supersedes their frame-selection policy.

- Search every native frame chronologically, without a preferred percentage or
  hard percentage cutoff.
- Select the earliest accepted qualifying frame independently for each method.
- BEV primary qualification: XY reference error strictly below 0.1 m.
- XYZ and `bev_lidar_height`: XYZ reference error strictly below 0.1 m.
- Also search a BEV XYZ benchmark using the same XYZ requirement, so horizontal
  success is not mistaken for accurate height.
- Preserve each method's original registration, upstream and fusion gates.
- Keep the existing per-vehicle height calibration and explicit map-ground
  assumptions; removing the search cutoff does not make the road globally flat.
- An unexamined/corrupt/missing frame makes a search incomplete. Only complete
  exhaustion permits the labelled inherited frame-zero fallback, which is not
  a newly accepted localization.

The selection uses recorded-pose/ENU reference errors offline. It is not a
deployment-ready confidence rule or independently surveyed ground-truth test.
Independent selection frames compare coverage and required replay advance;
their errors are not a matched-frame accuracy comparison or runtime benchmark.

Outputs are isolated in `artifacts/full-sequence-localization`. Immutable input,
map and configuration fingerprints validate resumable evaluations. Source MCAPs
remain read-only and the existing teleoperation queue is unchanged.

Use `--worker-id NAME --run-id SEQUENCE` for disjoint sequence workers. Use
`scripts/investigation/report_full_sequence_localization.py` to refresh the
comparison. The report distinguishes queued, searching, incomplete, exhausted
and qualified states; no final ranking is inferred while searches are unfinished.

[Live report](/tmp/RemoteTeleop-hybrid-road-height/artifacts/full-sequence-localization/report.md)
· [Numeric table](/tmp/RemoteTeleop-hybrid-road-height/artifacts/full-sequence-localization/report.csv).

Validation before launch: 22 targeted tests passed; two real-frame smoke cases
and fingerprint-validated resume passed. Existing results were retained.
