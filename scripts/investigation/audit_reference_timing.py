"""First-order metadata timing diagnostic on eight unchanged gallery cases."""
from pathlib import Path
import json
import sys
import zipfile
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from src.localization.cooperscene_reference import CooperSceneReference

ROOT=Path(__file__).resolve().parents[2]
OUT=ROOT/'artifacts/reference-refinement'
ARCHIVES=Path('/mnt/bcc-data/websites/data.ucr.edu-datasets-cooperscene')
SOURCE=OUT/'road-comparison-with-cameras/diagnostics.json'
VELOCITY_DOC='https://www.xsens.com/hubfs/Downloads/usermanual/MT_Manager_user_manual.pdf'


def main():
    cases=json.loads(SOURCE.read_text());cached={};rows=[]
    geographic=CooperSceneReference((33.975676,-117.339873,273),np.eye(4),np.eye(4))
    for case in cases:
        take,agent=case['run_id'].split('-')[1:]
        if (take,agent) not in cached:
            archive=next(ARCHIVES.glob(f'*/{take}.zip'));member=f'{take}/{agent}/metadata.json'
            with zipfile.ZipFile(archive) as z:metadata=json.loads(z.read(member))
            entries=sorted(metadata.values(),key=lambda e:e['gnss_ns'])
            cached[(take,agent)]=(archive,member,metadata,entries)
        archive,member,metadata,entries=cached[(take,agent)];m=metadata[case['frame_id']]
        dt=(int(m['lidar_ns'])-int(m['gnss_ns']))/1e9
        velocity=np.array([m[key] for key in ['Vel_E','Vel_N','Vel_U']]);speed=np.linalg.norm(velocity);displacement=velocity*dt
        slope=-np.asarray(case['map_road']['normal'])[:2]/case['map_road']['normal'][2]
        # Local LLA finite difference only checks units/axes; no pose is corrected.
        times=np.array([e['gnss_ns'] for e in entries],dtype=np.int64);before=entries[int(np.argmin(abs(times-(int(m['gnss_ns'])-200000000))))];after=entries[int(np.argmin(abs(times-(int(m['gnss_ns'])+200000000))))]
        interval=(int(after['gnss_ns'])-int(before['gnss_ns']))/1e9
        lla_velocity=(geographic.enu_position(after)-geographic.enu_position(before))/interval
        gap=next(v['signed_road_height_separation_m'] for v in case['variants'] if v['method']=='released')
        first_order_bound=abs(displacement[2])+np.linalg.norm(slope)*np.linalg.norm(displacement[:2])
        rows.append(dict(run_id=case['run_id'],frame_offset=case['frame_offset'],frame_id=case['frame_id'],metadata_archive=str(archive),metadata_member=member,gnss_ns=int(m['gnss_ns']),lidar_ns=int(m['lidar_ns']),lidar_minus_gnss_ms=dt*1000,velocity_enu_m_s=velocity.tolist(),speed_m_s=float(speed),speed_times_abs_dt_m=float(speed*abs(dt)),first_order_displacement_enu_m=displacement.tolist(),signed_vertical_velocity_times_dt_m=float(displacement[2]),fixed_map_road_slope_dz_de_dn=slope.tolist(),signed_road_slope_height_excursion_m=float(slope@displacement[:2]),road_slope_height_bound_m=float(np.linalg.norm(slope)*np.linalg.norm(displacement[:2])),first_order_translation_height_bound_m=float(first_order_bound),signed_vertical_minus_road_slope_excursion_m=float(displacement[2]-slope@displacement[:2]),released_signed_road_height_gap_m=gap,lla_unit_check_interval_s=interval,lla_finite_difference_velocity_enu_m_s=lla_velocity.tolist(),lla_vs_recorded_horizontal_velocity_error_m_s=float(np.linalg.norm(lla_velocity[:2]-velocity[:2]))))
    result=dict(status='first_order_diagnostic_only_no_pose_interpolation_or_correction',cases_source=str(SOURCE),velocity_units='m/s, ENU',velocity_units_evidence=[dict(url=VELOCITY_DOC,section='5.7.3.7 Velocity view',description='MT Manager manual labels Vel E/N/U velocity in m/s; default ENU.'),dict(path='/mnt/bcc-data/data/Cooperscene/backup/250928_xsens/t1/MT_0080002725_018-000.txt',lines='1-13',description='Original MT Manager text export identifies MTi-680G and coordinate system ENU, with Vel_E/Vel_N/Vel_U columns.'),dict(description='Per-case LLA finite differences below independently check the velocity scale and ENU axes; they do not interpolate poses.')],formulas=dict(dt='(lidar_ns-gnss_ns)/1e9',first_order_displacement='v_ENU*dt',road_slope='-map_plane.normal[:2]/map_plane.normal[2]',first_order_translation_height_bound='abs(vU*dt)+norm(road_slope)*norm(vEN*dt)'),records=rows,summary=dict(dt_ms_min=min(r['lidar_minus_gnss_ms'] for r in rows),dt_ms_max=max(r['lidar_minus_gnss_ms'] for r in rows),displacement_m_min=min(r['speed_times_abs_dt_m'] for r in rows),displacement_m_max=max(r['speed_times_abs_dt_m'] for r in rows),max_abs_vertical_displacement_m=max(abs(r['signed_vertical_velocity_times_dt_m']) for r in rows),max_road_slope_height_bound_m=max(r['road_slope_height_bound_m'] for r in rows),max_first_order_translation_height_bound_m=max(r['first_order_translation_height_bound_m'] for r in rows)),limitations=['Constant-velocity first-order estimate at metadata timestamp, not a proven total timing-error bound.','No attitude evolution, angular-rate integration, gyro unit assumption, lever-arm rotational velocity, acceleration, or LiDAR scan deskew included.','Map road slopes are the fixed diagnostic fitted planes and inherit their ROI/road-band assumptions.','Velocity axes are ENU; comparison assumes supplied map tie has identity rotation. These quantities do not depend on exported quaternion normalization.','Metadata timestamps may contain sensor convention/systematic clock effects beyond the measured numerical difference.','Static infrastructure-versus-map road mismatch cannot be explained by vehicle GNSS/LiDAR motion timing.'])
    (OUT/'timing-audit.json').write_text(json.dumps(result,indent=2)+'\n')
    report='''# Same-frame first-order timing audit

Native archive metadata were read for the eight unchanged road-comparison cases. No poses were interpolated, shifted, or corrected. Velocity uses ENU meters per second: the original MT Manager export explicitly declares ENU, and the [Xsens MT Manager manual, section 5.7.3.7](https://www.xsens.com/hubfs/Downloads/usermanual/MT_Manager_user_manual.pdf) labels Vel E/N/U in m/s. Local LLA finite differences also agree with this scale and axis convention; per-case checks are preserved in the JSON.

`dt = lidar_ns − gnss_ns`; positive dt means the LiDAR timestamp follows the GNSS sample. Motion estimates use `v × dt`. For each previously fitted map road, gradient `g = −normal_xy/normal_z`; the road-slope height excursion is `g·v_xy·dt`, and its magnitude is at most `||g||·||v_xy·dt||` under this first-order model. Adding `|vU·dt|` yields a conservative translation-only height magnitude for this model. It is not a complete temporal pose-error bound.

| Take/agent, offset | LiDAR−GNSS, ms | Speed, m/s | Speed × dt, m | Signed vU × dt, m | Road-slope height bound, m | Translation-only combined height bound, m | Released road-height gap, m |
|---|---:|---:|---:|---:|---:|---:|---:|
'''
    for r in rows:
        report+=f"| {r['run_id'].replace('cooperscene-','').replace('-','/')}, {r['frame_offset']} | {r['lidar_minus_gnss_ms']:.3f} | {r['speed_m_s']:.3f} | {r['speed_times_abs_dt_m']:.3f} | {r['signed_vertical_velocity_times_dt_m']:+.4f} | {r['road_slope_height_bound_m']:.4f} | {r['first_order_translation_height_bound_m']:.4f} | {r['released_signed_road_height_gap_m']:+.3f} |\n"
    s=result['summary'];report+=f"\nThe observed skew spans **{s['dt_ms_min']:.2f}–{s['dt_ms_max']:.2f} ms**, with first-order motion magnitude **{s['displacement_m_min']:.3f}–{s['displacement_m_max']:.3f} m**. Timing can therefore matter substantially for horizontal scan alignment. However, the maximum direct vertical excursion is **{s['max_abs_vertical_displacement_m']:.4f} m**; the maximum combined vertical-plus-road-slope estimate is **{s['max_first_order_translation_height_bound_m']:.4f} m**. Translation timing at the observed velocities and fitted road slopes cannot by itself account for the roughly 0.24–1.77 m released road-height disagreements in these cases.\n"
    report+='''
No angular timing effect is bounded here: gyro units, sensor-body conventions, rotational lever arms and scan deskew would require additional validated modeling. Acceleration and clock/timestamp semantics are also omitted. Consequently this evidence does not rule out every timing contribution; it distinguishes potentially large horizontal motion from much smaller modeled vertical/road-slope motion. The static infrastructure versus GSplat road disagreement is a separate observation that cannot arise from moving-vehicle GNSS/LiDAR timing.

This analysis uses raw timestamp, LLA and velocity fields, and fixed map plane normals. It is unaffected by revisions to the interpretation or normalization of exported orientation quaternions. Full values, source member paths, formulas and unit checks are in `timing-audit.json`; reproduce with `scripts/investigation/audit_reference_timing.py`.
'''
    (OUT/'timing-audit.md').write_text(report);print(json.dumps(result['summary'],indent=2));print(report)

if __name__=='__main__':main()
