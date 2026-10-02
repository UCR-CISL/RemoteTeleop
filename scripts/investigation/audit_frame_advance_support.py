"""Plot recorded sensor routes against robust map bounds, without fitting poses."""
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    first=Path('artifacts/first-frame-localization')
    output=Path('artifacts/frame-advance-localization')
    output.mkdir(parents=True,exist_ok=True)
    prior=Path('/tmp/RemoteTeleop-mcap-replay/artifacts/proxy-batch/xyz-results')
    rows=[]
    figure,axes=plt.subplots(5,3,figsize=(14,14),layout='constrained')
    for record in json.loads((first/'inputs/manifest.json').read_text()):
        if record['dataset']!='cooperscene':
            continue
        run=record['run_id']
        best=json.loads((first/'xyz/best'/f'{run}.json').read_text())
        bounds=np.asarray(best['coverage']['robust_map_bounds_dataset_xyz_m'])
        frame=np.load(record['npz_path'])
        reference=frame['reference_map_T_dataset']
        path=prior/run/'attempt-001/lambda/localization-trajectory.jsonl'
        data=[json.loads(line) for line in path.read_text().splitlines()]
        positions=np.asarray([(np.linalg.inv(reference)@np.asarray(item['reference_map_T_lidar']))[:3,3]
                              for item in data])
        seconds=np.asarray([item['timestamp_us'] for item in data])/1e6
        gap=np.linalg.norm(np.maximum(bounds[0,:2]-positions[:,:2],0)+
                           np.maximum(positions[:,:2]-bounds[1,:2],0),axis=1)
        inside=np.flatnonzero(gap==0)
        entry=int(inside[0]) if len(inside) else None
        rows.append(dict(run_id=run,total_frames=len(data),duration_seconds=float(seconds[-1]),
            first_sensor_inside_robust_xy_frame=entry,
            first_sensor_inside_seconds=float(seconds[entry]) if entry is not None else None,
            first_sensor_inside_fraction=float(seconds[entry]/seconds[-1]) if entry is not None else None,
            closest_frame=int(np.argmin(gap)),closest_xy_gap_m=float(gap.min()),initial_xy_gap_m=float(gap[0]),
            note='Sensor position versus robust map bounds only; does not establish scan overlap or registration acceptance'))
        axis=axes[int(record['scene'])-1,int(record['agent'])-1]
        axis.plot(seconds,gap,color='#285d8f')
        axis.axvspan(0,seconds[-1]*.15,color='#a6dba0',alpha=.2)
        axis.axvline(seconds[-1]*.25,color='#bf5b21',ls=':')
        if entry is not None:
            axis.axvline(seconds[entry],ls='--',color='#248641')
        axis.set(title=f"Take {record['scene']}, agent {record['agent']} — entry {seconds[entry]:.1f}s" if entry is not None else run,
                 xlabel='Seconds from first replay frame',ylabel='Sensor distance outside XY bounds (m)',ylim=(-1,max(5,gap.max()*1.1)))
        axis.grid(alpha=.2)
    figure.suptitle('CooperScene recorded routes and map support\nGreen: preferred 15%; orange: hard 25% cutoff; green dash: sensor entry. Bounds do not prove overlap.',fontsize=15)
    figure.savefig(output/'trajectory-map-support.png',dpi=130)
    plt.close(figure)
    (output/'trajectory-map-support.json').write_text(json.dumps(rows,indent=2)+'\n')


if __name__=='__main__':
    main()
