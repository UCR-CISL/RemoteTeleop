"""Report frame selection separately from localization algorithm changes."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


ROOT=Path('artifacts/frame-advance-localization')
FIRST=Path('artifacts/first-frame-localization')


def read(path):
    return json.loads(path.read_text())


def table(headers,rows):
    return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join('---' for _ in headers)+' |']+
        ['| '+' | '.join(str(value) for value in row)+' |' for row in rows])


def number(value,digits=3):
    return '—' if value is None else f'{value:.{digits}f}'


def load_attempts():
    rows=[]
    selections=[]
    certificates=[read(path) for path in (ROOT/'cooperscene/certified_exclusions').glob('*.json')]
    for dataset,directory in [('nuscenes','attempts'),('cooperscene','results')]:
        for path in sorted((ROOT/dataset/directory).glob('*/*.json')):
            result=read(path)
            original_id=(f"nuscenes-{result['scene']}" if dataset=='nuscenes' else result['original_run_id'])
            variant=result['variant']
            original=read(FIRST/'xyz'/variant/f'{original_id}.json')
            for key in ('initial_map_T_dataset','reference_map_T_dataset','registration_config','fit_config'):
                assert result[key]==original[key],(path,key,'changed settings')
            error=result.get('fresh_error',{})
            final=result.get('final_registration',{})
            scan=result.get('scan_registration',{})
            offset=result.get('sequence_index',result.get('frame_offset'))
            seconds=result.get('offset_seconds',result.get('elapsed_seconds'))
            row=dict(dataset=dataset,original_run_id=original_id,run_id=result['run_id'],variant=variant,
                frame_id=result['frame_id'],frame_offset=offset,elapsed_seconds=seconds,
                frame_count=result.get('frame_count',result.get('sequence_frame_count')),
                sequence_fraction=result['sequence_fraction'],scan_accepted=bool(scan.get('accepted',False)),
                sequence_percent=100*result['sequence_fraction'],within_preferred_window=result['sequence_fraction']<.15,
                within_hard_cutoff=result['sequence_fraction']<=.25,
                final_accepted=bool(result['accepted']),
                reference_error_below_target=bool(result['accepted'] and error.get('xyz_m',float('inf'))<.1),
                target_hit=bool(result['accepted'] and error.get('xyz_m',float('inf'))<.1 and result['sequence_fraction']<=.25),
                sampling_phase=result.get('sampling_phase',''),result_path=str(path),
                rejection_reasons=','.join(result.get('rejection_reasons',[])),exception=result.get('exception',''))
            for key in ('dx_m','dy_m','dz_m','xy_m','xyz_m','roll_deg','pitch_deg','yaw_deg','orientation_deg'):
                row[key]=error.get(key)
            for key in ('inlier_ratio','median_residual','p95_residual','all_point_median_residual','all_point_p95_residual'):
                row[key]=final.get(key)
            rows.append(row)
    assert len({(r['run_id'],r['variant']) for r in rows})==len(rows)
    for original_id,variant in sorted({(r['original_run_id'],r['variant']) for r in rows}):
        all_attempts=[r for r in rows if (r['original_run_id'],r['variant'])==(original_id,variant)]
        subset=sorted([r for r in all_attempts if r['within_hard_cutoff']],key=lambda r:r['frame_offset'])
        def earliest(key):
            return next((r for r in subset if r[key]),None)
        hit=earliest('target_hit')
        accepted=[r for r in subset if r['final_accepted']]
        best=min(accepted,key=lambda r:r['xyz_m']) if accepted else None
        failed=[r for r in subset if r['xyz_m'] is not None]
        selected=hit or best or (min(failed,key=lambda r:r['xyz_m']) if failed else None)
        exclusions=[r for r in certificates if r['original_run_id']==original_id and r['sequence_fraction']<=.25]
        examined_indices=sorted({r['frame_offset'] for r in subset}|{r['frame_offset'] for r in exclusions})
        selections.append(dict(original_run_id=original_id,dataset=subset[0]['dataset'],variant=variant,
            original_xyz_m=subset[0]['xyz_m'],original_accepted=subset[0]['final_accepted'],
            first_scan=earliest('scan_accepted'),first_final=earliest('final_accepted'),
            first_target=hit,best_accepted=best,selected_visual=selected,
            visual_reason='qualifying target' if hit else ('minimum accepted error diagnostic' if best else 'rejected diagnostic; no accepted fit'),
            globally_earliest_proven=bool(hit and [index for index in examined_indices if index<=hit['frame_offset']]==list(range(hit['frame_offset']+1))),
            attempt_count=len(subset),historical_attempts_beyond_cutoff=len(all_attempts)-len(subset),
            certified_exclusion_count=len(exclusions),
            eligible_window_exhausted=examined_indices==list(range(int(.25*(subset[0]['frame_count']-1))+1)),
            target_within_preferred_window=bool(hit and hit['within_preferred_window']),
            max_tested_seconds=max([r['elapsed_seconds'] for r in subset]+[r['elapsed_seconds'] for r in exclusions])))
    return rows,selections


def main():
    rows,selections=load_attempts()
    with (ROOT/'attempts.csv').open('w') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    (ROOT/'selections.json').write_text(json.dumps(selections,indent=2)+'\n')
    (ROOT/'settings-audit.json').write_text(json.dumps(dict(candidate_count=len(rows),
        unchanged_fields=['initial_map_T_dataset','reference_map_T_dataset','registration_config','fit_config'],
        all_match_original=True,selection_target='accepted XYZ reference disagreement <0.1m'),indent=2)+'\n')
    text=['# Advancing replay frames with unchanged localization',
        'This tests whether a later recorded frame permits an accepted XYZ result below 0.1 m. It does **not** improve the localization algorithm. '
        'Every tested frame is fitted independently with its original initializer, reference, masks, optimizer settings and acceptance gates. '
        'No warm start, reference reset, threshold relaxation or trajectory optimization is used.',
        'The main comparison is the existing baseline. nuScenes `static_fine` is reported separately using exactly the previously tested fixed method; '
        'it is not a new optimization. Distances remain disagreement with the ENU/recorded-pose reference, not independent ground-truth ATE.',
        '## Search and interpretation',
        '- **Preferred: before 15%; hard cutoff: at most 25% of sequence duration.** Percentage is 100 × frame offset / (frame count − 1). '
        'All choices and minima below exclude frames beyond 25%. Older exploratory attempts beyond the cutoff remain in the CSV and do not count.',
        '- Start with frame 0 and advance on a fixed 1-second grid, including the exact last eligible frame. '
        'For nuScenes, check every native 0.1-second frame before a selected success, or the entire eligible window when no success exists. '
        'For CooperScene, check every frame preceding a selected success, refine milestone brackets, and densely check no-hit cases with an accepted error below 0.15 m. '
        'Other CooperScene failures retain the fixed-grid sampling limitation.',
        '- Report scan acceptance, final XYZ acceptance and accepted XYZ below 0.1 m separately. Scan acceptance includes all original geometric/correction gates; it is stronger than a bounds-only overlap clue.',
        '- Selected successes are verified by checking their entire earlier eligible prefix. Exhaustive failure means every eligible frame was checked; '
        'a sampled failure leaves untested intervals. Proof and coverage flags are recorded in [selections.json](selections.json).',
        '- A failed grid search does not prove no frame in the sequence can succeed. The tested offsets and errors are preserved in [attempts.csv](attempts.csv). '
        '“Minimum accepted XYZ” is a descriptive minimum over searched frames, not an estimator selected without reference knowledge.',
        '- nuScenes already has accepted scan/final registration at frame 0 in all eight scenes. Its later-frame search tests reference agreement, not a need to acquire overlap.',
        '## Results']
    if any(item['certified_exclusion_count'] for item in selections):
        text.insert(-1,'Some CooperScene frames are excluded without ICP by an exact necessary-condition bound from the unchanged correction gates: '
            'any permitted correction moves initial-mapped point p by at most 1 m + 2 sin(3°/2) × ||p||. '
            'Full-cloud residual lower bounds can then prove the gates impossible. [Certificates](cooperscene/certified_exclusions) '
            'are separate from fits and cannot count as accepted poses.')
    outcomes=[]
    for dataset,variant in [('nuscenes','baseline'),('cooperscene','baseline'),('nuscenes','static_fine')]:
        group=[item for item in selections if (item['dataset'],item['variant'])==(dataset,variant)]
        outcomes.append([dataset,variant,len(group),sum(item['first_target'] is not None for item in group),
            sum(item['target_within_preferred_window'] for item in group),
            sum(item['first_target'] is None and item['eligible_window_exhausted'] for item in group),
            sum(item['first_target'] is None and not item['eligible_window_exhausted'] for item in group)])
    text.insert(3,'## Outcome\n\n'+table(['Dataset','Fixed method','Sequences','Qualify ≤25%','Qualify <15%',
        'Exhaustive no-hit windows','Sampled no-hit windows'],outcomes))
    text.append('Frame-0 values below are the fresh localization result on frame 0, not the inherited replay alignment. '
                'Errors for rejected fits describe unaccepted diagnostic candidates, including diverged ICP estimates; '
                'they are not valid replay poses. The original report retains the inherited-alignment comparison.')
    for dataset,variant,title in [('nuscenes','baseline','nuScenes — unchanged baseline'),
                                  ('cooperscene','baseline','CooperScene — unchanged baseline'),
                                  ('nuscenes','static_fine','nuScenes — previously fixed masked/finer method')]:
        text.append('### '+title)
        display=[]
        for item in selections:
            if (item['dataset'],item['variant'])!=(dataset,variant):continue
            hit=item['first_target'] or {};best=item['best_accepted'] or {}
            display.append([item['original_run_id'],number(item['original_xyz_m'])+(' accepted' if item['original_accepted'] else ' rejected'),
                number((item['first_scan'] or {}).get('elapsed_seconds'),1),
                number((item['first_final'] or {}).get('elapsed_seconds'),1),
                hit.get('frame_id','Not found (all eligible frames checked)' if item['eligible_window_exhausted'] else 'Not found (sampled)'),number(hit.get('elapsed_seconds'),1),
                number(100*hit['sequence_fraction'],2)+'%' if hit else '—',number(hit.get('xyz_m')),
                'Preferred' if hit and hit['within_preferred_window'] else ('Within hard cutoff' if hit else 'No eligible hit'),
                number(best.get('xyz_m')),f"{item['attempt_count']} fits + {item['certified_exclusion_count']} excluded / through {item['max_tested_seconds']:.1f}s"])
        text.append(table(['Sequence','Frame-0 fresh XYZ (m)','First scan accepted (s)','First XYZ accepted (s)',
            'First observed target frame','Delay (s)','Sequence elapsed','Target XYZ (m)','Window','Minimum accepted XYZ (m)','Eligible tested frames'],display))
    text.append('## Baseline cases without an eligible qualifying frame')
    text.append('For reference, these are the lowest accepted errors found within the cutoff. Where no fit was accepted, '
                'the lowest rejected-candidate error is shown explicitly as a diagnostic. Rejected estimates do not establish valid localization.')
    failures=[]
    for item in selections:
        if item['variant']!='baseline' or item['first_target'] is not None:
            continue
        candidate=item['selected_visual'] or {}
        failures.append([item['original_run_id'],number(candidate.get('xyz_m')),number(candidate.get('xy_m')),
            number(candidate.get('dz_m')),number(candidate.get('sequence_percent'),2)+'%',
            'Accepted, misses target' if candidate.get('final_accepted') else 'REJECTED diagnostic',
            'Every eligible frame' if item['eligible_window_exhausted'] else 'Sampled frames'])
    text.append(table(['Sequence','XYZ (m)','XY (m)','Signed Z (m)','Sequence elapsed','Status','Coverage'],failures))
    orientation=[]
    for item in selections:
        if item['dataset']=='cooperscene' and item['first_target'] is not None:
            orientation.append(f"{item['original_run_id']}: {item['first_target']['orientation_deg']:.2f}°")
    if orientation:
        text.append('The qualifying CooperScene frames retain reference-relative orientation disagreement ('+
                    '; '.join(orientation)+'). Advancing the start satisfies the translation target at those frames; '
                    'the map/reference tilt remains. No full-trajectory run or optimization was performed.')
    bev=[]
    for path in sorted((ROOT/'nuscenes/bev_selected').glob('*.json')):
        item=read(path)
        bev.append([item['scene'],item['variant'],item['frame_id'],number(item['sequence_percentage'],2)+'%',
                    number(item['error']['xy_m']),str(item['accepted']),str(item['target_hit'])])
    if bev:
        text+=['## BEV at the XYZ-selected later frames',
            'These are secondary evaluations at the same chosen frames, not a separate BEV frame search. '
            'nuScenes BEV retains the initializer’s vertical position and gravity, so its zero vertical difference is imposed.',
            table(['Scene','Fixed method','Frame','Sequence elapsed','BEV XY (m)','Accepted','Below 0.1 m'],bev)]
    text+=['## Evidence and reproducibility',
        '[Original first-frame report in Markdown](../first-frame-localization/report.md) · '
        '[Settings invariants](settings-audit.json) · [Selection records](selections.json) · '
        '[Later-frame source and reader audit](later-frame-native-audit.json)',
        '![Recorded sensor positions relative to robust map bounds](trajectory-map-support.png)',
        'This route plot uses the existing recorded reference poses. Sensor entry into the robust 1–99% map XY bounds does not prove sufficient LiDAR overlap.',
        '![All attempted baseline XYZ fits](baseline-search.png)']
    original_ids=[r['run_id'] for r in read(FIRST/'inputs/manifest.json')]
    figure,axes=plt.subplots(6,4,figsize=(16,17),layout='constrained')
    for axis,run_id in zip(axes.flat,original_ids):
        subset=[r for r in rows if r['original_run_id']==run_id and r['variant']=='baseline' and r['xyz_m'] is not None and r['within_hard_cutoff']]
        for accepted,color,marker,label in [(True,'#267244','o','Accepted'),(False,'#c04b42','x','Rejected')]:
            group=[r for r in subset if r['final_accepted']==accepted]
            axis.scatter([r['sequence_percent'] for r in group],[max(r['xyz_m'],.001) for r in group],s=14,c=color,marker=marker,label=label)
        axis.axhline(.1,color='black',ls='--',lw=.8)
        axis.axvspan(0,15,color='#a6dba0',alpha=.15)
        axis.set(title=run_id,xlabel='Sequence elapsed (%)',ylabel='XYZ difference (m)',yscale='log',xlim=(0,25))
        axis.grid(alpha=.2)
    axes.flat[-1].axis('off')
    figure.suptitle('Frame search with unchanged baseline localization\nDashed line: 0.1m; rejected candidates never count as target hits. Logarithmic error axis.')
    figure.savefig(ROOT/'baseline-search.png',dpi=130);plt.close(figure)
    for item in selections:
        stem=f"{item['original_run_id']}-{item['variant']}"
        camera=ROOT/'visuals'/f'{stem}-camera.png'
        if camera.exists():
            text.append(f"### {stem}\n\n{item['visual_reason']}.\n\n![Recorded cameras, fresh GSplat renders and overlays](visuals/{stem}-camera.png)\n\n"
                        f"![Annotation-masked LiDAR versus map](visuals/{stem}-cloud.png)")
    (ROOT/'report.md').write_text('\n\n'.join(text)+'\n')
    print(f'Reported {len(rows)} candidates across {len(selections)} sequence/method pairs.')


if __name__=='__main__':
    main()
