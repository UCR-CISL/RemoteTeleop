"""Compare fixed physical patches through ±2s without changing released poses."""
from pathlib import Path
import argparse
import hashlib
import json
import zipfile
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from compare_selected_lidar_structures import StructureAgreement, DEFAULT, ROOT
from select_infra_overlap_frames import InfrastructureOverlapSelector, voxel

OUT = ROOT / 'artifacts/reference-refinement/structure-experiment/temporal'


def prepare(output):
    selector = InfrastructureOverlapSelector()
    manifest = json.loads((ROOT / 'artifacts/reference-refinement/overlap-selected-calibration/selection/manifest.json').read_text())
    rows, clouds = [], {}
    for chosen in manifest['global_selected']:
        take, agent = chosen['take'], chosen['agent']
        with zipfile.ZipFile(chosen['archive']) as archive:
            metadata = json.loads(archive.read(f'{take}/{agent}/metadata.json'))
            frames = sorted(n.split('/')[-1][:-4] for n in archive.namelist()
                            if n.startswith(f'{take}/{agent}/') and n.endswith('.pcd'))
            for offset in range(max(0, chosen['frame_offset']-20), min(len(frames), chosen['frame_offset']+21)):
                frame = frames[offset]
                points, pose, _ = selector.read_scan(archive, take, agent, frame)
                key = f't{take}_a{agent}_f{offset}'
                clouds[key] = voxel(points, .20)
                m = metadata[frame]
                rows.append(dict(key=key, take=take, agent=agent, frame_id=frame, frame_offset=offset,
                                 selected_offset=chosen['frame_offset'], relative_time_s=(int(m['lidar_ns'])-chosen['lidar_ns'])/1e9,
                                 metadata=m, pose=pose.tolist(), archive=chosen['archive']))
        print(f'Prepared take {take} / agent {agent}', flush=True)
    np.savez_compressed(output/'window-clouds.npz', **clouds)
    (output/'window-manifest.json').write_text(json.dumps(rows, indent=2)+'\n')


def measure(output):
    experiment = StructureAgreement(DEFAULT)
    rows = json.loads((output/'window-manifest.json').read_text())
    windows = np.load(output/'window-clouds.npz')
    infra = {roi['id']:experiment.fit(roi, 'infra_representative')[1] for roi in experiment.rois}
    results = []
    (output/'protocol.json').write_text(json.dumps(dict(roi_manifest_sha256=hashlib.sha256((DEFAULT/'roi-manifest.json').read_bytes()).hexdigest(), fixed_central_anchor=True, pose_corrections=False, full_point_timing_available=False),indent=2)+'\n')
    W = experiment.clouds['original_W']
    for row in rows:
        experiment.clouds['window'] = windows[row['key']]
        m = row['metadata']; pose = W @ np.asarray(row['pose'])
        yaw = Rotation.from_matrix(pose[:3,:3]).as_euler('xyz', degrees=True)[2]
        for roi in experiment.rois:
            record, fit = experiment.fit(roi, 'window')
            base = infra[roi['id']]
            uv = np.zeros((1,2))  # One unchanged physical location across time and sensors.
            supported = base is not None and fit is not None and bool(experiment.anchor_supported(base, uv)[0] and experiment.anchor_supported(fit, uv)[0])
            r = dict(key=row['key'], take=row['take'], agent=row['agent'], frame_id=row['frame_id'],
                     frame_offset=row['frame_offset'], relative_time_s=row['relative_time_s'], roi=roi['id'],
                     supported=supported, vehicle_surface=record, lidar_minus_gnss_ms=(int(m['lidar_ns'])-int(m['gnss_ns']))/1e6,
                     speed_m_s=float(np.linalg.norm([m['Vel_E'],m['Vel_N'],m['Vel_U']])), yaw_deg=float(yaw))
            if supported:
                b=experiment.basis(roi); anchor=np.asarray(roi['anchor'],float)
                anchor -= (anchor@base['n']+base['d'])/(b[2]@base['n'])*b[2]
                separation=-(anchor@fit['n']+fit['d'])/(fit['n']@base['n'])
                sensor_point=(anchor-pose[:3,3])@pose[:3,:3]
                r.update(signed_orthogonal_separation_m=float(separation), anchor_xyz=anchor.tolist(),
                         normal_angle_deg=float(np.degrees(np.arccos(np.clip(base['n']@fit['n'],-1,1)))),
                         anchor_sensor_azimuth_deg=float(np.degrees(np.arctan2(sensor_point[1],sensor_point[0]))))
            results.append(r)
    (output/'measurements.json').write_text(json.dumps(results,indent=2)+'\n')
    fig,axes=plt.subplots(3,3,figsize=(18,12),sharex=True)
    summaries=[]
    for roi,ax in zip(experiment.rois,axes.flat):
        for agent,color in [(1,'#d62728'),(2,'#2877bd'),(3,'#16a05d')]:
            rr=[r for r in results if r['roi']==roi['id'] and r['agent']==agent]
            good=[r for r in rr if r['supported']]
            ax.plot([r['relative_time_s'] for r in rr],[r.get('signed_orthogonal_separation_m',np.nan) for r in rr],'.-',color=color,label=f'Vehicle {agent}')
            summary=dict(roi=roi['id'],agent=agent,supported_frames=len(good),total_frames=len(rr))
            if good:
                val=np.array([r['signed_orthogonal_separation_m'] for r in good])
                summary.update(median_signed_m=float(np.median(val)),minimum_signed_m=float(min(val)),maximum_signed_m=float(max(val)),
                               maximum_normal_angle_deg=max(r['normal_angle_deg'] for r in good),
                               yaw_span_deg=float(np.ptp(np.degrees(np.unwrap(np.radians([r['yaw_deg'] for r in good]))))),
                               speed_range_m_s=[min(r['speed_m_s'] for r in good),max(r['speed_m_s'] for r in good)])
            summaries.append(summary)
        ax.axhline(0,color='gray',lw=.6);ax.axvline(0,color='gray',ls=':');ax.set(title=roi['id'],xlabel='Seconds from selected scan',ylabel='Vehicle − infra along infra normal (m)');ax.grid(alpha=.25)
    axes.flat[0].legend();fig.suptitle('Fixed released poses; SAME central physical anchor over time; missing support leaves gaps')
    fig.tight_layout();fig.savefig(output/'fixed-anchor-time-windows.png',dpi=150);plt.close(fig)
    (output/'summary.json').write_text(json.dumps(summaries,indent=2)+'\n')
    text=['# Fixed-structure temporal check','','123 original scans: 41 per selected vehicle, nominal ±2 seconds. Every released pose stays fixed. Each curve evaluates the same central anchor on the stationary infrastructure plane, only where both clouds have supported planes, hull containment and nearby samples. Missing support is not agreement or disagreement. Dynamic annotation masks and 0.20m voxel sampling match the selected-cloud experiment.','','| Structure | Vehicle | Supported / total | Median signed gap (m) | Signed range (m) | Max normal angle (deg) |','|---|---|---:|---:|---|---:|']
    for r in summaries:
        text.append(f"| {r['roi']} | {r['agent']} | {r['supported_frames']}/{r['total_frames']} | {r.get('median_signed_m',float('nan')):.3f} | {r.get('minimum_signed_m',float('nan')):.3f} .. {r.get('maximum_signed_m',float('nan')):.3f} | {r.get('maximum_normal_angle_deg',float('nan')):.2f} |")
    text+=['','## Interpretation limits','','Surface gaps, vehicle heading, speed, timestamp differences and anchor azimuth in sensor coordinates are recorded together. Azimuth is geometry, not a measured per-point acquisition time. Correlation cannot isolate clock offset, scan distortion, calibration, or changing sampling. No deskew or timing correction is estimated. The short windows can reveal persistence/change but cannot establish general take-wide consistency. No production localization acceptance thresholds change.']
    (output/'report.md').write_text('\n'.join(text)+'\n')

    fig,axs=plt.subplots(3,3,figsize=(16,13));picked=[]
    for i,name in enumerate(['NW_upper_facade','SW_boundary_wall','road_central']):
     roi=next(x for x in experiment.rois if x['id']==name);rr=[r for r in results if r['agent']==3 and r['roi']==name and r['supported']]
     extremes=[min(rr,key=lambda r:r['signed_orthogonal_separation_m']),min(rr,key=lambda r:abs(r['relative_time_s'])),max(rr,key=lambda r:r['signed_orthogonal_separation_m'])]
     for j,r in enumerate(extremes):
      picked.append(r);b=experiment.basis(roi);ax=axs[i,j]
      for p,c,label in [(experiment.clouds['infra_representative'],'black','Infra'),(windows[r['key']],'#16a05d','Vehicle3')]:
       local=(p-roi['anchor'])@b.T;h=np.array(roi['half_widths_uvn']);h[2]=1.3
       local=local[np.all(abs(local)<=h,axis=1)];ax.scatter(local[:,0],local[:,2],s=9,c=c,label=label,alpha=.65)
      ax.axhline(roi['half_widths_uvn'][2],c='gray',ls='--');ax.axhline(-roi['half_widths_uvn'][2],c='gray',ls='--')
      ax.set(title=f"{name}\n{r['key']} t={r['relative_time_s']:+.2f}s gap={r['signed_orthogonal_separation_m']:+.3f}m",xlabel='Tangent u (m)',ylabel='Declared normal distance (m)',ylim=(-1.3,1.3));ax.grid(alpha=.2)
    axs[0,0].legend();fig.suptitle('Temporal extreme audit: expanded normal crop; dashed lines delimit fixed fit selection slab')
    fig.tight_layout();fig.savefig(output/'temporal-extreme-crops.png',dpi=150)
    (output/'temporal-extreme-crops.json').write_text(json.dumps(picked,indent=2))
    plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,default=OUT);parser.add_argument('--prepare',action='store_true')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    if args.prepare: prepare(args.output)
    measure(args.output)
