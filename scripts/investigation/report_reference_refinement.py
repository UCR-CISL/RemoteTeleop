"""Build the report and visual index from the fixed-frame reference audit."""
from pathlib import Path
import json,csv,html
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[2]
out=root/'artifacts/reference-refinement'
records=json.loads((out/'final-comparison/diagnostics.json').read_text())
labels={'released':'Released ICP','physical':'Normalized INS + body calibration (conditional)','map_tied_release':'Released + frozen map tie','xyz':'Existing XYZ','bev_lidar_height':'Existing BEV + height'}
colors={'released':'#1698b5','physical':'#289a53','map_tied_release':'#aa8800','xyz':'#de5625','bev_lidar_height':'#9c4abd'}
rows=[]
for r in records:
 for v in r['variants']:
  rows.append(dict(run_id=r['run_id'],frame_offset=r['frame_offset'],method=v['method'],label=labels[v['method']],saved_registration_accepted=v['saved_registration_accepted'],normal_angle_deg=v['normal_angle_deg'],signed_road_height_separation_m=v['signed_road_height_separation_m'],road_comparison_supported=v['road_comparison_supported'],road_footprint_overlap_2m=v['road_footprint_overlap_2m'],comparison_x_m=r['comparison_xy_map'][0],comparison_y_m=r['comparison_xy_map'][1],source_fit_p95_m=r['source_road']['quality']['p95_residual_m'],map_fit_p95_m=r['map_road']['quality']['p95_residual_m']))
with (out/'road-comparison.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
fig,axes=plt.subplots(4,2,figsize=(15,17))
for ax,r in zip(axes.flat,records):
 x=np.linspace(-12,12,100);xy=np.repeat(np.array(r['comparison_xy_map'])[None,:],100,axis=0);xy[:,0]+=x
 mp=r['map_road'];mn=np.array(mp['normal']);map_z=-(xy@mn[:2]+mp['offset'])/mn[2]
 ax.plot(x,map_z,'k--',lw=2,label='GSplat road plane')
 for v in r['variants']:
  n=np.array(v['normal']);z=-(xy@n[:2]+v['offset'])/n[2]
  reject=' (rejected)' if v['saved_registration_accepted'] is False else ''
  ax.plot(x,z,lw=1.6,c=colors[v['method']],label=labels[v['method']]+reject)
 ax.axvline(0,color='gray',lw=.7);ax.grid(alpha=.2)
 rejected=[labels[v['method']] for v in r['variants'] if v['saved_registration_accepted'] is False]
 ax.set(xlabel='Map X from fixed comparison point (m)',ylabel='Map Z (m)',title=r['run_id']+' / frame '+str(r['frame_offset'])+('\nRejected diagnostic: '+', '.join(rejected) if rejected else ''))
handles,legend=axes.flat[0].get_legend_handles_labels()
fig.subplots_adjust(top=.89,bottom=.04,left=.07,right=.98,hspace=.40,wspace=.22)
fig.legend(handles,legend,loc='upper center',bbox_to_anchor=(.5,.958),ncol=3,fontsize=9)
fig.suptitle('Measured road-plane cuts at the same map Y for each case\nLines are fitted surfaces, not localization ATE or ground truth; full 3D normal angles are in the report',fontsize=13,y=.995)
fig.savefig(out/'road-plane-overview.png',dpi=150);plt.close(fig)
report='''# CooperScene reference refinement: findings and limits

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

See [release provenance and composition details](release-audit/FINDINGS.md). The conditional normalized-sensor helper was checked against all 6,705 newly exported poses. Old artifacts are preserved, not silently replaced.

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
'''
for r in records:
 vals=[]
 for v in r['variants']:
  suffix=' †' if v['saved_registration_accepted'] is False else ''
  vals.append(f"{v['normal_angle_deg']:.2f}° / {v['signed_road_height_separation_m']:+.3f}{suffix}")
 report+='| '+r['run_id'].replace('cooperscene-','').replace('-','/')+', '+str(r['frame_offset'])+' | '+' | '.join(vals)+' |\n'
report+='''
† Saved localization failed its original registration acceptance. No gate is relaxed and no pose is promoted because a road plane looks good.

All eight map/source plane pairs pass the reported common-location hull and footprint support checks. Map-road candidates use the documented broad map Z band **[-4.3, -1.3] m**, radius 15 m, and existing plane-fit configuration; the band is a semantic diagnostic assumption, not a supplied GT height. Plane normals are fitted, not forced to gravity. Hulls can bridge gaps, local roads can curve, and Gaussian centers need not describe a single layer perfectly. Full residual/support details are in [road-comparison.csv](road-comparison.csv) and each case’s JSON/NPZ.

## Infrastructure debugging evidence

At the same supported road location, infrastructure and GSplat road normals disagree by **2.36–2.78°**, and infrastructure road height is **0.259–0.326 m above** the GSplat. This static mismatch cannot be caused by moving-vehicle timestamp skew. It demonstrates that aligning vehicles to infrastructure does not automatically align them to this GSplat.

The infrastructure’s `raw_gnss.csv` is byte-identical to agent 1’s in all five takes. Its fixed pose has no independently verified survey provenance in the available files. Infrastructure geometry is independent of GSplat localization, but is **not independent validation of the released ICP**, which was fitted to it. It is therefore useful diagnostic evidence, not absolute GT.

See [infrastructure audit](infrastructure-audit/report.md). Its older physical-INS comparison is explicitly marked superseded; the infrastructure planes and release-versus-pre-ICP evidence remain valid.

## Vehicle-only fixed road/map calibration experiment

One calibration frame was declared before testing: **take 1 / agent 1, frame 433 (native ID 481693)**, the original ICP anchor, excluded from the eight comparison frames. Only its vehicle LiDAR and GSplat road planes were used. One transform was frozen: roll **−0.586°**, pitch **+2.661°**, yaw **0°**, translation **[0, 0, −1.719] m**, about the fixed map origin. This is a coordinate-frame correction, not a per-frame height adjustment or the road height itself.

| Held-out metric, median across 8 frames | Released reference | Fixed map tie | Frames improved |
|---|---:|---:|---:|
| Road normal disagreement | 3.258° | 1.182° | 8/8 |
| Absolute road-height separation | 0.631 m | 0.461 m | 4/8 |
| Full static-cloud NN median residual | 0.363 m | 0.294 m | 6/8 |
| Full static-cloud NN p95 residual | 1.106 m | 0.701 m | 8/8 |

This is evidence for a shared orientation mismatch, **not a complete GT correction**. Four height errors and two full-cloud median residuals worsened. The frozen correction is map-calibrated and must not be called independent truth. See [calibration and held-out details](vehicle-road-calibration/report.md).

A separate source-only hypothesis preserves the original ICP anchor while changing sensor relative motion (`K = C B(anchor) inverse(P(anchor))`). Even with corrected metadata normalization it does not consistently improve the eight road comparisons. It remains an unpromoted diagnostic; do not simply reuse numeric C on differently composed sensor poses.

## Timing, slope and remaining disagreement

For these eight frames, LiDAR–GNSS skew is **4.85–73.67 ms**, corresponding to **0.044–0.804 m** first-order motion. Timing can substantially affect horizontal alignment. However, the largest modeled vertical-plus-road-slope translation contribution is only **0.0197 m**, much smaller than the released road-height gaps of **0.235–1.774 m**. Translation timing alone therefore cannot explain those gaps. Angular timing, smoothing, lever-arm effects and scan deskew remain unbounded. See [timing calculations](timing-audit.md).

The remaining discrepancy cannot yet be assigned uniquely among local road geometry, infrastructure/map frame calibration, marker-to-sensor axes/origins, vehicle attitude bias, GNSS elevation/datum, and timing. Better full-cloud ICP alignment can compensate these effects; it does not establish an independent reference. A plane constrains tilt and one height relation but does not independently resolve in-plane XY/yaw.

## Recommendation

1. Keep the **released refined reference** and the correctly decoded **conditional INS/sensor reference** distinct. Do not promote the previous metadata-based “physical correction.”
2. Treat the frozen vehicle-only map tie as a useful orientation diagnostic, not a new universal reference: residual height errors are still too large and inconsistent.
3. For a defensible map-relative benchmark, establish one map-to-release calibration from designated, geometrically diverse **vehicle** calibration frames, and test it on held-out vehicles/takes. Use static non-road structures as well as road surfaces; do not fit each evaluation frame to manufacture GT agreement.
4. Validate device/marker axes, GNSS reference point/altitude datum and capture-time poses before interpreting decimeter errors as physical accuracy. Propagate the verified frame consistently into priors and evaluation references.
5. Until those checks pass, report **registration consistency and reference disagreement**, not ground-truth accuracy, and do not re-rank XYZ versus BEV + height using the earlier <0.1 m counts.

No full teleoperation batch was launched. Existing queues, original results, acceptance gates and mounted source files were preserved.
'''
(out/'report.md').write_text(report)
(root/'docs/investigations/reference-refinement.md').write_text(report.replace('(release-audit/','(../../artifacts/reference-refinement/release-audit/').replace('(road-comparison.csv)','(../../artifacts/reference-refinement/road-comparison.csv)').replace('(infrastructure-audit/','(../../artifacts/reference-refinement/infrastructure-audit/').replace('(vehicle-road-calibration/','(../../artifacts/reference-refinement/vehicle-road-calibration/').replace('(timing-audit.md)','(../../artifacts/reference-refinement/timing-audit.md)'))
body=['<!doctype html><html><head><meta charset="utf-8"><title>CooperScene reference comparison</title><style>body{font:17px sans-serif;max-width:1600px;margin:30px auto;color:#222}img{max-width:100%}section{margin:50px 0}table{border-collapse:collapse}td,th{padding:8px;border:1px solid #ccc}</style></head><body><h1>CooperScene reference comparison — no candidate is certified GT</h1><p>The previous “corrected GT” interpretation is superseded: metadata quaternions were already transformed. Released ICP is intentional refinement. These views compare the same frame and LiDAR points, with both XYZ and BEV + height named explicitly.</p><p><a href="report.md">Findings and recommendation</a> | <a href="road-comparison.csv">Road measurements CSV</a> | <a href="road-plane-overview.png">Road-plane overview PNG</a></p><p>Rows: released ICP; normalized INS + conditional body calibration; released + one frozen vehicle-only map tie; existing XYZ; existing BEV + height. Rejected estimates remain labelled. No acceptance gates changed. Infrastructure was debugging evidence only.</p><img src="road-plane-overview.png">']
for r in records:
 stem=f"{r['run_id']}-f{r['frame_offset']:06d}";body.append(f'<section><h2>{html.escape(stem)}</h2><h3>Same-frame point clouds and measured plane cuts</h3><a href="final-comparison/{stem}/clouds.png"><img src="final-comparison/{stem}/clouds.png"></a><h3>Recorded camera, GSplat render, 50% overlay</h3><a href="final-comparison/{stem}/camera-overlays.png"><img src="final-comparison/{stem}/camera-overlays.png"></a><p><a href="final-comparison/{stem}/diagnostics.json">Fit/support diagnostics</a> | <a href="final-comparison/{stem}/road-support.npz">Road support points</a></p></section>')
(out/'index.html').write_text('\n'.join(body+['</body></html>']))
for rel in ['docs/investigations/reference-orientation.md','docs/cooperscene_reference_orientation_audit.md']:
 p=root/rel;s=p.read_text();banner='> **Superseded by the release-refinement audit.** The metadata quaternion was already preprocessed. The earlier claim that only position was transformed, and the interpretation of the old physical reference as a correction, are withdrawn. See '+('[the updated investigation](reference-refinement.md).' if '/investigations/' in rel else '[the updated investigation](investigations/reference-refinement.md).')+'\n\n'
 if not s.startswith('> **Superseded'):p.write_text(banner+s)
print('Report, labelled gallery, CSV and road-plane PNG generated.')
