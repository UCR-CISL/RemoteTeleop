import numpy as np
import pytest

from src.realtime.bevfusion_sweeps import PastSweepAssembler


def test_past_scan_compensates_translation_rotation_and_preserves_feature_semantics():
    assembler = PastSweepAssembler()
    old_pose = np.eye(4)
    old_pose[:2,:2] = [[0,-1],[1,0]]
    old_pose[:3,3] = [10,0,2]
    old = np.array([[2,0,1,255],[.5,.5,0,22],[1,.5,1,11]],dtype=np.float32)
    first = assembler.add(old,old_pose,0,'take1V1')
    np.testing.assert_array_equal(first[:,:4],old)
    assert np.all(first[:,4]==0)
    current_pose = np.eye(4)
    current_pose[:3,3] = [9,1,1]
    current = np.array([[.1,.1,0,2800]],dtype=np.float32)
    result = assembler.add(current,current_pose,100_000,'take1V1')
    np.testing.assert_array_equal(result[:1,:4],current)  # Current near-origin point is kept.
    np.testing.assert_array_equal(result[1:,:3],[[1,1,2],[.5,0,2]])
    np.testing.assert_array_equal(result[:,3],[2800,255,11])
    np.testing.assert_allclose(result[:,4],[0,.1,.1])
    assert result.dtype==np.float32 and result.flags.c_contiguous
    assert assembler.last_past_timestamps_us==(0,)


def test_bounded_history_age_inclusive_and_start_boundary_no_future_or_padding():
    assembler = PastSweepAssembler(maximum_past_sweeps=2,maximum_age_seconds=.2)
    points=np.array([[2,0,0,10]],dtype=np.float32)
    outputs=[assembler.add(points,np.eye(4),ts,'s') for ts in (0,100_000,200_000,300_000)]
    assert [len(result) for result in outputs]==[1,2,3,3]
    np.testing.assert_allclose(outputs[-1][:,4],[0,.1,.2])
    assert assembler.last_past_timestamps_us==(200_000,100_000)
    result=assembler.add(points,np.eye(4),500_001,'s')
    assert len(result)==1 and assembler.last_past_timestamps_us==()


def test_nine_past_scans_maximum_with_distinct_capture_ages():
    assembler=PastSweepAssembler()
    points=np.array([[2,0,0,7]],dtype=np.float32)
    for index in range(12):result=assembler.add(points,np.eye(4),index*100_000,'s')
    assert len(result)==10
    np.testing.assert_allclose(result[:,4],np.arange(10)*.1)
    assert len(assembler.last_past_timestamps_us)==9


def test_zero_history_single_scan_and_scene_reset_allow_earlier_new_capture():
    points=np.array([[0,0,0,1]],dtype=np.float32)
    assembler=PastSweepAssembler(maximum_past_sweeps=0)
    for timestamp in (0,100_000):
        result=assembler.add(points,np.eye(4),timestamp,'s')
        np.testing.assert_array_equal(result,[[0,0,0,1,0]])
    assembler=PastSweepAssembler()
    assembler.add(np.array([[2,0,0,1]],dtype=np.float32),np.eye(4),100_000,'old')
    assert len(assembler.add(points,np.eye(4),0,'new'))==1
    assembler.reset()
    assert len(assembler.add(points,np.eye(4),0,'new'))==1


def test_history_owns_copies_and_output_mutations_do_not_change_source_or_history():
    assembler=PastSweepAssembler()
    points=np.array([[2,0,0,10]],dtype=np.float32);pose=np.eye(4)
    original=points.copy(); original_pose=pose.copy()
    output=assembler.add(points,pose,0,'s')
    np.testing.assert_array_equal(points,original); np.testing.assert_array_equal(pose,original_pose)
    output[:]=999;points[:]=888;pose[:3,3]=777
    result=assembler.add(original,original_pose,100_000,'s')
    np.testing.assert_array_equal(result[:,:4],np.concatenate([original,original]))


@pytest.mark.parametrize('timestamp',[0,-1])
def test_duplicate_and_backwards_captures_are_rejected_without_losing_history(timestamp):
    assembler=PastSweepAssembler();points=np.array([[2,0,0,10]],dtype=np.float32)
    assembler.add(points,np.eye(4),0,'s')
    with pytest.raises(ValueError,match='strictly increase'):assembler.add(points,np.eye(4),timestamp,'s')
    assert len(assembler.add(points,np.eye(4),100_000,'s'))==2


@pytest.mark.parametrize('kwargs',[{'maximum_past_sweeps':-1},{'maximum_past_sweeps':10},
    {'maximum_past_sweeps':True},{'maximum_past_sweeps':1.5},{'maximum_age_seconds':0},
    {'maximum_age_seconds':float('nan')},{'maximum_age_seconds':float('inf')}])
def test_invalid_configuration_rejected(kwargs):
    with pytest.raises(ValueError):PastSweepAssembler(**kwargs)


def test_invalid_scan_or_pose_rejected_before_sequence_reset():
    assembler=PastSweepAssembler();points=np.array([[2,0,0,1]],dtype=np.float32)
    assembler.add(points,np.eye(4),0,'s')
    with pytest.raises(ValueError,match='Nx4'):assembler.add(np.ones((1,5)),np.eye(4),1,'new')
    invalid=np.eye(4);invalid[0,0]=2
    with pytest.raises(ValueError,match='orthonormal'):assembler.add(points,invalid,1,'new')
    assert len(assembler.add(points,np.eye(4),100_000,'s'))==2
