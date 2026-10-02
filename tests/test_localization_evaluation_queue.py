import csv
import json
from pathlib import PurePosixPath

from scripts.deployment.prepare_localization_evaluation import LocalizationEvaluationPreparer
from scripts.deployment.run_remote_teleop_batch import ReplayBatch
from scripts.deployment.run_remote_teleop_sam_simulation import SamRemoteTeleopSimulationLauncher


def test_best_xyz_uses_minimum_accepted_baseline_not_earliest_hit(tmp_path):
    fields = ['variant', 'final_accepted', 'within_hard_cutoff', 'sequence_percent', 'xyz_m', 'frame_offset', 'original_run_id', 'result_path']
    result = tmp_path / 'fit.json'
    result.write_text(json.dumps(dict(final_accepted=True, map_T_dataset=[[1, 0, 0, 0]])))
    rows = []
    for variant, accepted, percent, error, offset in [('baseline', True, 0, .09, 0), ('baseline', True, 10, .02, 10), ('baseline', False, 12, .001, 12), ('static_fine', True, 13, .0001, 13), ('baseline', True, 30, .00001, 30)]:
        rows.append(dict(zip(fields, [variant, str(accepted), 'True', percent, error, offset, 'test', str(result)])))
    attempts = tmp_path / 'attempts.csv'
    with attempts.open('w') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    assert LocalizationEvaluationPreparer.best_xyz(attempts)['test']['frame_offset'] == 10


def test_selected_queue_plumbs_start_ground_and_bevfusion(tmp_path):
    base = tmp_path / 'base.json'
    base.write_text(json.dumps(dict(version=1, runs=[dict(id='scene', dataset='cooperscene', scene='1', agent='2', mcap='data/1.mcap', splat='data/map.ply', expected_samples=100)])))
    out = tmp_path / 'queue'
    builder = LocalizationEvaluationPreparer(base, out, PurePosixPath('/inputs/eval'))
    builder.prepare({'scene': dict(frame_offset=12, error_m=.02, map_T_dataset=[[1, 0, 0, 0]], source_result='fit.json')}, mode='xy', bbox_source='bevfusion', config=__import__('pathlib').Path('cfg/alien3_lambda_proxy_batch.yaml'), ground_planes={'scene': '/maps/plane.json'})
    value = json.loads((out / 'manifest.json').read_text())
    # CooperScene requires a stored transform alongside the selected one.
    value['runs'][0]['cooperscene_stored_localization_transform'] = '/inputs/identity.json'
    (out / 'manifest.json').write_text(json.dumps(value))
    batch = ReplayBatch(out / 'manifest.json', out / 'results')
    options, sam = batch._options(batch.runs[0], 1)
    assert options.expected_samples == 88
    assert options.mcap_start_sequence == 12
    assert options.bbox_source == 'bevfusion'
    launcher = SamRemoteTeleopSimulationLauncher(options, sam, dry_run=True)
    assert '--mcap-start-sequence 12' in launcher._vehicle_adapter_script()
    assert '--ground-plane /maps/plane.json' in launcher._remote_start_script()
    assert value['evaluation']['coverage'][0]['status'] == 'qualified'


def test_no_subthreshold_fit_preserves_original_frame_zero_transform(tmp_path):
    base = tmp_path / 'base.json'
    original = dict(id='scene', dataset='nuscenes', scene='152', mcap='in.mcap', splat='map.ply', expected_samples=100, nuscenes_localization_transform='/prior.json')
    base.write_text(json.dumps(dict(version=1, runs=[original])))
    builder = LocalizationEvaluationPreparer(base, tmp_path / 'out', PurePosixPath('/new'))
    result = builder.prepare({'scene': dict(frame_offset=12, error_m=.1, map_T_dataset=[], source_result='bad.json')}, mode='xyz', bbox_source='bevfusion', config=__import__('pathlib').Path('cfg/alien3_lambda_proxy_batch.yaml'))
    assert result['runs'][0]['nuscenes_localization_transform'] == '/prior.json'
    assert result['runs'][0]['mcap_start_sequence'] == 0
    assert result['runs'][0]['expected_samples'] == 100
    assert result['evaluation']['coverage'][0]['status'] == 'fallback_frame_zero_inherited_transform'


def test_bev_inherited_fallback_is_never_relabelled_qualified(tmp_path):
    selections = tmp_path / 'selections.json'
    selections.write_text(json.dumps({'sequences': [dict(run_id='scene', search_complete=True, selected=dict(accepted=True, fallback_existing_alignment=True, error={'xy_m': 0.0}))]}))
    assert LocalizationEvaluationPreparer.best_bev(selections) == {}
