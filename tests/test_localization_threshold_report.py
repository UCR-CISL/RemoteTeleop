from scripts.investigation.report_localization_thresholds import summarize_frames


def attempt(frame, error, accepted=True):
    return dict(frame_offset=frame, elapsed_seconds=frame / 10, sequence_percent=frame,
                methods={'xyz': dict(accepted=accepted, error={'xyz_m': error})})


def test_rejected_low_error_does_not_qualify_and_threshold_is_strict():
    result = summarize_frames([attempt(0, .001, False), attempt(1, .15), attempt(2, .09)], 'xyz', 'xyz_m', 8)
    assert result['best_rejected_diagnostic']['frame_offset'] == 0
    assert result['thresholds']['0.15']['selected']['frame_offset'] == 2
    assert result['thresholds']['0.20']['selected']['frame_offset'] == 1
    assert result['best_scope'] == 'evaluated_frames_only'


def test_only_full_coverage_proves_no_candidate():
    assert summarize_frames([attempt(0, .3)], 'xyz', 'xyz_m', 2)['thresholds']['0.10']['status'] == 'unresolved'
    result = summarize_frames([attempt(0, .3), attempt(1, .2)], 'xyz', 'xyz_m', 2)
    assert result['thresholds']['0.10']['status'] == 'exhausted'
    assert result['best_scope'] == 'entire_sequence'


def test_gap_prevents_earliest_frame_claim():
    result = summarize_frames([attempt(0, .3), attempt(2, .05)], 'xyz', 'xyz_m', 3)
    assert result['thresholds']['0.10']['status'] == 'unresolved'
