# CooperScene-to-GSplat alignment per take and sensor

## What the visualization shows

Every frame is placed with the full composed transform
`map_T_lidar = D @ W @ released_ENU_T_lidar`. `D` is the provisional shared tie
from `shared-enu-calibration/calibration.json`, and released poses are
unchanged. Each LiDAR point is compared with the GSplat surface along its
normal. The surface is opacity > 0.05, local-centroid plane, local thickness
< 0.10 m. The comparison covers 599 cached frames over all five takes and all
four sensors.

The transform holds for the stationary infrastructure in every take. For the
vehicles it holds only in some takes and at some times. At the 0.1 m level it is
not valid across takes and vehicles.

## Summary per take and sensor

Median over frames of |road height, LiDAR minus map|. The percentage is the
share of frames under 0.1 m.

| Take | Infrastructure | Vehicle 1 | Vehicle 2 | Vehicle 3 |
|---|---|---|---|---|
| 1 | 0.04 m, 100% | 0.09 m, 56% | 0.14 m, 16% | 0.35 m, 9% |
| 2 | 0.03 m, 100% | 0.14 m, 12% | 0.03 m, 99% | 0.15 m, 31% |
| 3 | 0.04 m, 100% | 0.14 m, 0% | 0.24 m, 7% | 0.19 m, 6% |
| 4 | 0.03 m, 100% | 0.19 m, 29% | 0.11 m, 45% | 0.03 m, 100% |
| 5 | 0.04 m, 100% | 0.52 m, 13% | 0.47 m, 0% | 0.30 m, 23% |

Median wall separation is 0.16–0.17 m for the infrastructure in every take and
0.19–0.37 m for the vehicles. The GSplat surface is itself about 0.35 m thick,
so wall separations below about 0.1–0.2 m cannot be resolved.

## Patterns visible in the per-take figures

- **Infrastructure is consistent.** It sits within 0.03–0.04 m in road height
  in every take, at every sampled time.
- **Some vehicles carry steady offsets.** In take 2, vehicles 1 and 3 hold about
  −0.14 m and +0.30 m for roughly 80 s while vehicle 2 stays within 0.05 m.
  In take 5, vehicle 2 sits about 0.47 m below the map for its whole sequence.
- **Some vehicles move off the map quickly.** In take 5, vehicle 1 ramps from
  −0.55 m to +0.35 m over 18 s. Vehicle 3 swings between −0.7 m and +0.6 m
  within 10 s.
- **Shared-transform tilt adds to it.** Vehicle error measured against the map
  includes any tilt error in the shared `D` at the vehicle's location. Take 5
  vehicles drive mostly in the east of the scene, far from the infrastructure.
  The earlier LiDAR-to-LiDAR analyses separate the two sources. Vehicle poses
  disagree with the infrastructure regardless of `D`.

Everything here is geometric agreement with the GSplat map, not independent
ground truth.

## Artifacts and reproduction

Everything is in `artifacts/reference-refinement/take-vehicle-map-alignment/`.
`index.html` holds the table and figures. `figures/take-sensor-summary.png` is
the 5×4 summary grid. `figures/take{1..5}-alignment.png` hold, per sensor, a top
view coloured by signed distance plus per-frame road and wall residuals over
time. `measurements.json` holds every frame's numbers.

```bash
OMP_NUM_THREADS=4 /home/jyue86/Documents/CISL-Projects/RemoteTeleop/.venv/bin/python \
  scripts/investigation/render_take_vehicle_map_alignment.py
```
