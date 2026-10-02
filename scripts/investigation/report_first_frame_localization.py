"""Build a reviewable first-frame report and complete per-candidate metric table."""
from __future__ import annotations
import csv
from html import escape
import json
from pathlib import Path


ROOT=Path('artifacts/first-frame-localization')


def read(path):
    return json.loads(path.read_text())


def metric_row(metadata, mode, variant, error, registration, accepted, refusal=''):
    row=dict(run_id=metadata['run_id'],dataset=metadata['dataset'],scene=metadata['scene'],
        agent=metadata['agent'],frame_id=metadata['frame_id'],mode=mode,variant=variant,
        accepted=bool(accepted),independent_ground_truth=False)
    for key in ('dx_m','dy_m','dz_m','xy_m','xyz_m','roll_deg','pitch_deg','yaw_deg','orientation_deg'):
        row[key]=error.get(key)
    for key in ('inlier_ratio','median_residual','p95_residual','all_point_median_residual',
                'all_point_p95_residual','correction_translation','correction_rotation_degrees'):
        row[key]=registration.get(key)
    row['rejection_reasons']=','.join(registration.get('rejection_reasons',[]))+('; '+refusal if refusal else '')
    return row


def table(headers, rows):
    return '<table><thead><tr>'+''.join(f'<th>{escape(h)}</th>' for h in headers)+'</tr></thead><tbody>'+''.join(
        '<tr>'+''.join(f'<td>{v}</td>' for v in row)+'</tr>' for row in rows)+'</tbody></table>'


def number(value):
    return '—' if value is None else f'{value:.3f}'


def status(value):
    return '<b class="accepted">accepted</b>' if value else '<b class="rejected">REJECTED</b>'


def main():
    metadata=read(ROOT/'inputs/manifest.json')
    rows=[]
    for item in metadata:
        run_id=item['run_id']
        baseline=read(ROOT/'xyz/baseline'/f'{run_id}.json')
        rows.append(metric_row(item,'inherited','first_replay_frame',baseline['inherited_error'],
            baseline['inherited_registration'],baseline['inherited_registration']['accepted']))
        for path in sorted((ROOT/'xyz').glob(f'*/{run_id}.json')):
            if path.parent.name=='best':
                continue
            result=read(path)
            registration=result.get('final_registration',result.get('scan_registration',{}))
            additional=[reason for reason in result.get('rejection_reasons',[])
                        if reason not in registration.get('rejection_reasons',[])]
            rows.append(metric_row(item,'xyz',path.parent.name,result.get('fresh_error',{}),registration,
                result.get('accepted',False),'; '.join(additional)))
        for path in sorted((ROOT/'bev').glob(f'{run_id}_*.json')):
            result=read(path)
            refusal=result.get('refusal','')
            if not result.get('upstream_scan_accepted',True):
                refusal+='; upstream scan rejected'
            if not result.get('fusion_converged',True):
                refusal+='; fusion not converged'
            rows.append(metric_row(item,'bev',result['variant'],result.get('error',{}),
                result.get('registration',{}),result.get('accepted',False),refusal))
    with (ROOT/'metrics.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    content=['''<!doctype html><html><head><meta charset="utf-8"><title>First-frame localization investigation</title>
    <style>body{font:15px system-ui,sans-serif;max-width:1500px;margin:32px auto;padding:0 20px;color:#18212b}
    table{border-collapse:collapse;width:100%;margin:18px 0;font-size:13px}th,td{padding:8px;text-align:right;border-bottom:1px solid #ddd}
    th{background:#edf2f6;position:sticky;top:0}td:first-child,th:first-child{text-align:left}img{max-width:100%}
    .accepted{color:#166534}.rejected{color:#b91c1c}.note{padding:16px;background:#fff4d6}details{margin:20px 0}code{background:#eee}</style></head><body>
    <h1>First-frame localization: 23 sequences</h1>
    <p>Eight nuScenes scenes with their own GSplats; CooperScene takes 1–5, agents 1–3, with the latest Riverside map.
    Offline first frames only. Mounted sources were read without modification. Previous replay results are preserved.</p>
    <p class="note"><b>These are reference-relative pose differences, not independent ground-truth ATE.</b>
    One frame measures absolute position disagreement; it cannot establish trajectory accuracy.
    Rejected estimates are diagnostic candidates. The target of accepted XYZ disagreement below 0.1 m is not met for all sequences.</p>
    <h2>Definitions and audit</h2>
    <ul><li><b>Inherited:</b> the first replay pose using the prior batch alignment (nuScenes frame 100 fit; CooperScene take 1/agent 1 fit).
    <b>Fresh:</b> registration using only this sequence’s first LiDAR frame, followed by the configured fusion where applicable.</li>
    <li>The faithful CooperScene baseline still starts from the historical coarse initializer. A separately labelled <code>static_global</code>
    variant computes fresh first-frame FPFH/RANSAC initialization, then applies the original strict refinement gates.</li>
    <li>nuScenes baseline is full SE(3) ICP. CooperScene XYZ fusion optimizes XYZ translation and yaw while retaining scan roll/pitch.
    CooperScene BEV fusion optimizes XY/yaw and retains scan Z/roll/pitch; both use 3D associations.</li>
    <li>nuScenes BEV is an explicit experimental comparison: horizontal translation and gravity-axis yaw only, pivoted at the first LiDAR.
    Its zero vertical and roll/pitch differences are imposed by the recorded-pose initializer, not measured accuracy.</li>
    <li>Signed XYZ differences are resolved in dataset Z-up axes, not the camera-oriented nuScenes map axes.
    RPY is the relative spatial rotation expressed in those axes, decomposed with XYZ Euler angles in degrees.</li>
    <li>All 15 CooperScene cached LiDAR arrays exactly match native PCD points. Native YAML poses agree within 1.23e-13;
    camera-from-LiDAR agrees with documented per-agent calibration within 1.96e-14; intrinsics match exactly.
    This validates conversion consistency, not independent calibration accuracy.</li>
    <li>CooperScene pose order is meters and [roll, yaw, pitch] in degrees using the established OpenCOOD/CARLA rotation formula.
    CV camera coordinates are right/down/forward; dataset_T_camera = dataset_T_lidar × inverse(camera_T_lidar).</li>
    <li>All eight local nuScenes GSplat files match SHA256 hashes of their scene-specific Lambda checkpoint exports.
    Riverside SHA256 is f8731969529063e37a42f92ff3a9dc75b713bc1dc792bef41b0cd88f0701262d.</li>
    <li>The ENU reference is a shared translation with identity rotation. Its -0.430001 m vertical shift encodes the configured origin-height difference;
    the historical CooperScene height datum is not independently established. The nuScenes reference is the existing recorded-pose/map anchor.</li></ul>
    <h2>Dynamic returns and opacity</h2>
    <p>CooperScene already excludes annotated vehicle boxes padded by 0.2 m on each side. nuScenes baseline keeps all returns;
    <code>static</code> excludes vehicle, pedestrian and animal boxes, preserving barriers and cones. This also removes parked vehicles;
    incomplete annotations can leave dynamic returns. “Static” is an experiment label, not proof of static geometry.</p>
    <p>Fine registration previously required opacity &gt; 0, finite centers and coordinates within ±10 km. Only the CooperScene coarse map used opacity &gt; 0.05.
    Opacity measures transparency, not learned static confidence. The opacity-0.05 ablation is explicit and does not change acceptance thresholds.
    Riverside retains 538,458 of 802,305 centers at this cutoff. Some nuScenes camera renders still show vehicle surfaces in the GSplat.</p>
    <h2>Unchanged acceptance gates</h2>
    <p>nuScenes: inliers ≥0.20, inlier median ≤0.35 m, p95 ≤0.80 m, correction translation ≤3 m and rotation ≤30°.
    CooperScene strict: inliers ≥0.35, inlier median ≤0.20 m, p95 ≤0.60 m, all-point median ≤0.25 m and p95 ≤0.75 m,
    correction translation ≤1 m and rotation ≤3°. Global initialization uses the existing coarse stage; strict correction gates are evaluated against that actual coarse initializer.
    No scan points were cropped to improve acceptance. Display plots may crop the viewing region.</p>
    <p>Inlier residuals are conditional on the original correspondence radius (0.5 m nuScenes; 0.2 m CooperScene).
    All-point residuals include every retained source point; map-center proximity is not a surface or ground-truth error.
    nuScenes all-point median/p95 gates are disabled (infinite limits). Its 0.80 m limit applies only to inlier p95.
    Correction translation is the norm of the translation of T_candidate × inverse(T_initializer), not the first-LiDAR displacement; it depends on the coordinate origin.</p>
    <h2>Results by sequence</h2><p>Distances are meters. Main table uses faithful baseline settings; detailed tables and CSV include every fixed ablation.
    Baseline Coop already masks annotated vehicles. Rejected XYZ values can be pre-fusion scan candidates because the pipeline correctly refuses fusion after a rejected scan.</p>''']
    for dataset,title in [('nuscenes','nuScenes scenes'),('cooperscene','CooperScene takes / agents')]:
        content.append(f'<h3>{title}</h3>')
        display=[]
        for item in metadata:
            if item['dataset']!=dataset: continue
            selected=[r for r in rows if r['run_id']==item['run_id']]
            inherited=next(r for r in selected if r['mode']=='inherited')
            xyz=next(r for r in selected if r['mode']=='xyz' and r['variant']=='baseline')
            bev=next(r for r in selected if r['mode']=='bev' and r['variant']=='baseline')
            display.append([f'<a href="#{item["run_id"]}">{item["run_id"]}</a>',escape(item['frame_id']),
                number(inherited['xyz_m']),number(xyz['dx_m']),number(xyz['dy_m']),number(xyz['dz_m']),
                number(xyz['xy_m']),number(xyz['xyz_m']),status(xyz['accepted']),
                number(bev['xy_m']),number(bev['xyz_m']),status(bev['accepted'])])
        content.append(table(['Sequence','First frame','Inherited XYZ','Fresh ΔX','ΔY','ΔZ','Fresh XY','Fresh XYZ','XYZ status','BEV XY','BEV XYZ','BEV status'],display))
    content.append('<h2>Fixed experiment counts</h2>')
    counts=[]
    for dataset in ('nuscenes','cooperscene'):
        for mode in ('xyz','bev'):
            for variant in ('baseline','static','static_fine','static_global','static_opacity05'):
                subset=[r for r in rows if r['dataset']==dataset and r['mode']==mode and r['variant']==variant]
                if not subset: continue
                accepted=[r for r in subset if r['accepted']]
                counts.append([dataset,mode,variant,str(len(subset)),str(len(accepted)),
                    str(sum(r['xyz_m']<.1 for r in accepted)),str(sum(r['xy_m']<.1 for r in accepted))])
    content.append(table(['Dataset','Mode','Variant','Attempted','Accepted','Accepted XYZ <0.1','Accepted XY <0.1'],counts))
    content.append('''<h2>Findings and recommended corrections</h2><ol>
    <li><b>CooperScene vertical disagreement is strongly position-dependent.</b> The inherited rotation relative to ENU is 3.762°.
    Its vertical difference is exactly -0.0605262 x -0.0252535 y -0.0021529 z -1.924155 m for dataset LiDAR position (x,y,z).
    This explains why a transform near take1/agent1 becomes vertically far from the reference elsewhere. It does not establish which frame is correct.
    XYZ fusion cannot remove retained scan tilt; BEV also cannot correct scan Z.</li>
    <li><b>Map support is insufficient for many first scans.</b> The robust 1–99% Riverside XY bounds are approximately [-81.5,-80.8] to [69.4,70.5] m in reference dataset axes.
    Take4/agent2 lies roughly 53 m outside, with about 0.2% of its retained scan inside those bounds. Bounds are a coverage diagnostic, not exact overlap.
    Fresh global initialization did not rescue the rejected XYZ sequences. Preserve refusal until a supported pose/map region exists.</li>
    <li><b>Exact ENU agreement is not geometric validation.</b> Every CooperScene reference transform fails the current full-scan geometric gates.
    Even take1/agent1 has reference all-point median 0.293 m and p95 1.049 m versus fresh XYZ 0.214/0.665 m.
    Forcing the pose to ENU would manufacture zero reference error while degrading this map fit.</li>
    <li><b>Establish map/reference roll, pitch and height datum using surveyed static correspondences spread across the intersection.</b>
    Confirm whether a rigid georeference or a dataset calibration correction is justified before changing the production transform.
    A constant height offset alone cannot explain the observed spatial slopes. Native conversion checks rule out a newly introduced MCAP pose/point mismatch.</li>
    <li><b>Use explicit source masks and verify the map’s static content.</b> Keep baseline and masked metrics separate.
    Opacity alone is insufficient; investigate vehicle-contaminated map regions and map support before selecting a stronger opacity cutoff.
    A local supported-region gate could be investigated separately, but must not silently replace the current whole-scan gates.</li>
    <li><b>nuScenes refinement is promising but not uniformly better.</b> The fixed finer schedule changes voxels to 0.5/0.25/0.10 m and iteration cap to100;
    acceptance is reevaluated with original settings. Do not select a candidate by its closeness to the reference.
    The XYZ “best” directory ranks accepted candidates by common masked full-map all-point median then p95, not reference error.
    That policy yields 9/23 accepted and only 2/23 below 0.1 m XYZ, despite the fixed fine variant reaching 5/8 nuScenes.
    Scene171 illustrates the tradeoff: masking removes 11,373/34,720 points; masked fine fitting lowers all-point median/p95
    from 0.567/0.997 to 0.476/0.879 m while worsening reference XYZ disagreement from 0.571 to 0.751 m.
    All these nuScenes candidates pass the original gates. Scene529 also shows that the voxelized robust point-to-plane objective
    does not necessarily improve the reported Euclidean center-distance diagnostics. Neither scene supplies evidence sufficient to diagnose faulty calibration.</li>
    <li><b>Do not rerun the full teleoperation batch yet.</b> The present evidence does not justify a universal correction achieving &lt;0.1 m.
    BEV’s nuScenes vertical constraints and CooperScene’s ENU prior mean reference agreement is partly imposed.</li></ol>
    <h2>Visual validation</h2><p>All23 first-frame camera panels use actual CUDA GSplat renders: recorded image, rendered map and 50/50 overlay;
    rows show inherited, reference, fresh XYZ baseline and fresh BEV baseline. Rejected candidates are labelled.
    nuScenes171 retains a large truck surface; CooperScene1/3 shows poor map reconstruction/support, so image blur limits pose judgement.
    Recomputing CooperScene1/1 from the same first frame with faithful settings reproduces the earlier accepted transform to 7.2e-12; equality was not imposed.
    Camera alignment and LiDAR/map fit are complementary diagnostics; neither independently establishes ground-truth accuracy.</p>
    <p><a href="metrics.csv">Complete numeric CSV</a> · <a href="input-audit.json">Native input / opacity audit</a> ·
    <a href="inputs/manifest.json">First-frame and map manifest</a> · <a href="map-pairing-audit.json">Map pairing hashes</a> ·
    <a href="xyz/summary.json">XYZ residual-ranked summary</a></p>''')
    for item in metadata:
        run_id=item['run_id']
        content.append(f'<details id="{run_id}"><summary><b>{run_id}</b>: all metrics and visual evidence</summary>')
        display=[]
        for row in [r for r in rows if r['run_id']==run_id]:
            display.append([escape(row['mode']+'/'+row['variant']),status(row['accepted'])]+[
                number(row[k]) for k in ('dx_m','dy_m','dz_m','xy_m','xyz_m','roll_deg','pitch_deg','yaw_deg',
                'inlier_ratio','median_residual','p95_residual','all_point_median_residual','all_point_p95_residual')]+[escape(row['rejection_reasons'])])
        content.append(table(['Method','Status','ΔX','ΔY','ΔZ','XY','XYZ','Roll°','Pitch°','Yaw°','Inliers','Median','P95','All median','All P95','Refusal'],display))
        for relative in (f'camera-overlays/{run_id}.png',f'xyz/cloud_views/{run_id}.png',f'bev/{run_id}_static_fine.png'):
            if (ROOT/relative).exists():
                content.append(f'<p><a href="{relative}">{relative}</a></p><img loading="lazy" src="{relative}" alt="{escape(relative)}">')
        content.append('</details>')
    content.append('</body></html>')
    (ROOT/'report.html').write_text('\n'.join(content))
    print(f'{len(rows)} candidate rows; report: {ROOT}/report.html')


if __name__=='__main__':
    main()
