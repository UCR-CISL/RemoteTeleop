# BEV + LiDAR height: separate experimental method

Method identifier: `bev_lidar_height`. This is an offline experimental method;
existing BEV and XYZ implementations and deployment defaults are unchanged.

## Definition

1. Run the existing BEV localization stage on the chosen scan.
2. Fit a local road plane to annotation-masked LiDAR using the existing fit gates.
3. Obtain sensor clearance perpendicular to that road plane.
4. Given a configured map-road plane `n·p+d=0`, translate the BEV sensor pose
   along its frozen up axis `u` by
   `alpha = (clearance - (n·p+d)) / (n·u)`.
5. Keep BEV rotation and horizontal coordinates unchanged. Apply the same rigid
   correction to the camera through its full current-frame relative transform.
6. Apply the original upstream and full-cloud registration acceptance gates.
   Retain rejected proposals as diagnostics, with explicit unaccepted fallback.

The nominal-height comparator uses a separate calibration for each CooperScene
vehicle. It is reported separately from the current-scan measured-height method.
Reference errors never choose a height or determine acceptance.

## Explicit ground assumptions

- Riverside: native map Z = −2.8 m, tested as a flat-road assumption inherited
  from the rendering convention. It is not independently validated terrain.
- nuScenes: derive each scene's initial road plane from its first LiDAR scan
  and the known first-camera map anchor. Later use of that plane is explicitly
  an extrapolation experiment; the road is not assumed globally planar as fact.

Metric scale and accurate map-road geometry are accepted assumptions. They do
not remove local road slope or establish a universal map-road elevation.
Sensor localization and rendering-only asset grounding remain separate.

## Evaluation

All 23 first frames plus the previously selected nonzero BEV start frames;
no new frame search. Fresh BEV refinement/fusion is compared with the height
method on identical inputs. Any reused same-frame scan fit is recorded with
its provenance. Earlier initialization/ENU-fusion assumptions remain in place,
so reference agreement is not independent ground-truth accuracy.

Results are preserved separately under
`artifacts/bev-lidar-height` in `/tmp/RemoteTeleop-hybrid-road-height`.
The existing replay queue is not modified.

## Completed comparison

35 cases attempted: 34 comparisons and one explicit upstream BEV fusion failure
(CooperScene 4/2 frame 0, fewer than three correspondences). No accepted
measured-height correction improved XYZ reference agreement in this run.

| Group | Median XYZ, BEV → applied height | Accepted XYZ <0.1 m, BEV → height |
|---|---:|---:|
| nuScenes first frames (8) | 0.0825 → 0.0825 m | 5 → 5 |
| nuScenes selected frames (8) | 0.0102 → 0.1048 m | 8 → 4 |
| CooperScene first frames (14 available) | 5.5785 → 5.5785 m | 0 → 0 |
| CooperScene selected frames (4) | 0.3766 → 0.4770 m | 0 → 0 |

Medians include explicit fallback poses, not only accepted estimates. Initial
plane extrapolation and the assumed Riverside road datum remain limitations.
Keep this experimental; no new replay batch was launched. 33 targeted tests
passed, including 8 tests for the new method.

[Full report and per-frame tables](/tmp/RemoteTeleop-hybrid-road-height/artifacts/bev-lidar-height/report.md)
· [Comparison plot](/tmp/RemoteTeleop-hybrid-road-height/artifacts/bev-lidar-height/comparison.png)
· [Full metrics](/tmp/RemoteTeleop-hybrid-road-height/artifacts/bev-lidar-height/metrics.csv).

Runner: `scripts/investigation/run_bev_lidar_height.py --output <new-directory>`.
Use a new output directory to preserve previous results. Plot with
`python -m src.viz.plot_bev_lidar_height --directory <new-directory>`.
