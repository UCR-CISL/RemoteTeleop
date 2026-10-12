"""Opt-in current-capture publication, without sockets, ROS or inference."""
from types import SimpleNamespace

import numpy as np
import pytest

from src.deployment.ros_frame_adapter import McapFrameReplay, LatestBevFusionBoxes, RosFrameAdapter, RosFrameAdapterConfig, RosAnalysisAssembler
from src.deployment.vehicle import VehicleAnalysisSpool, VehicleFrameSpool
from src.realtime.bevfusion import BevFusionBox, BevFusionResult


class Client:
    def __init__(self):self.pending=None;self.requests=[];self.polls=0
    def submit(self,q):
        assert self.pending is None
        self.pending=q;self.requests.append(q);self.polls=0
        return True
    def poll(self):
        if self.pending is None:return None
        self.polls+=1
        if self.polls<2:return None
        q=self.pending;self.pending=None
        return BevFusionResult(q.request_id,q.scene_generation,q.sequence,q.frame_id,q.timestamp_us,q.config_id,
            (BevFusionBox((0.,0.,10.),(4.,2.,1.8),0.,.8,0,'vehicle'),))


@pytest.mark.parametrize('await_current',[False,True])
@pytest.mark.parametrize('via_replay',[False,True])
def test_current_result_publication_real_spool_and_no_double_submission(tmp_path,await_current,via_replay):
    client=Client();latest=LatestBevFusionBoxes(client,score_threshold=.2,minimum_area_px=.01)
    snapshots=VehicleFrameSpool(tmp_path/'snapshots');analysis=VehicleAnalysisSpool(tmp_path/'analysis');assembler=RosAnalysisAssembler(analysis,expected_samples=2)
    class Publisher:
        def send(self,*a,**kw):pass
    adapter=RosFrameAdapter(Publisher(),RosFrameAdapterConfig(bbox_source='bevfusion'),snapshot_sink=snapshots,analysis_assembler=assembler,bevfusion=latest)
    class BoundSidecar:
        start_index=0
        frames=[None,None]
        def bind(self,index,metadata,image,points):return metadata,points
    replay=McapFrameReplay(adapter,image_topic='/image',metadata_topic='/metadata',expected_samples=2,
        xyz_pose_sidecar=BoundSidecar(),await_detections=await_current,camera_calibration=None)
    pose=np.eye(4);pose[2,3]=1.234;K=np.array([[4.,0.,4.],[0.,4.,4.],[0.,0.,1.]])
    for sequence,t in enumerate((1_000_000,1_100_003)):
        header=SimpleNamespace(stamp=SimpleNamespace(sec=t//1_000_000,nanosec=t%1_000_000*1000))
        image=SimpleNamespace(header=header,height=8,width=8,step=24,encoding='bgr8',data=bytes(8*8*3))
        metadata=dict(version=1,frame_id=str(sequence),request_id=f'scene:{sequence}',scene_generation='scene',timestamp_us=t,
            world_T_ego=pose.ravel().tolist(),map_T_lidar=pose.ravel().tolist(),world_T_camera=pose.ravel().tolist(),camera_intrinsic=K.ravel().tolist(),end_of_scene=sequence==1)
        points=np.array([[0.,0.,1.,5.]],dtype=np.float32)
        if via_replay:
            replay._deliver_xyz(sequence,metadata,np.zeros((8,8,3),dtype=np.uint8),points)
        else:
            assembler.receive_image(image);latest.receive_image(image);latest.receive_lidar(SimpleNamespace(header=header,points=points))
            adapter.receive_metadata(metadata,wait_for_bevfusion=await_current)
        snapshot=snapshots.get('scene',sequence)
        assert snapshot.timestamp_us==t
        if await_current:
            assert len(snapshot.boxes)==1 and snapshot.boxes[0].track_id=='bevfusion:vehicle:0'
            assert snapshot.boxes[0].detection_confidence==.8
            assert np.asarray(snapshot.boxes[0].world_T_object).reshape(4,4)[2,3]==11.234
        elif sequence==0:assert snapshot.boxes==()
    if not await_current:
        latest.wait_for_sequence(1)
    assembler.close()
    assert len(client.requests)==latest.submitted_count==latest.result_count==2
    assert latest.replaced_pending_count==latest.error_count==0
    assert not latest._metadata and not latest._images and not latest._lidar
    assert not latest._in_flight and latest._pending is None
    assert analysis.get('scene',0).timestamp_us==1_000_000
    assert len(analysis.get('scene',0).boxes)==int(await_current)


def test_wait_timeout_and_client_error_are_bounded():
    class Silent:
        def poll(self):return None
    latest=LatestBevFusionBoxes(Silent(),score_threshold=.2)
    clock=[0.];sleeps=[]
    def sleep(dt):clock[0]+=dt;sleeps.append(dt)
    with pytest.raises(TimeoutError):latest.wait_for_sequence(0,timeout_seconds=.025,monotonic=lambda:clock[0],sleep=sleep)
    assert clock[0]==pytest.approx(.025) and max(sleeps)<=.01
    class Error:
        def poll(self):return dict(request_id='bad',error='model failed')
    latest=LatestBevFusionBoxes(Error(),score_threshold=.2)
    with pytest.raises(RuntimeError,match='failed'):latest.wait_for_sequence(0,sleep=lambda dt:pytest.fail('error must not sleep'))
    for value in (0,float('inf'),float('nan')):
        with pytest.raises(ValueError):latest.wait_for_sequence(0,timeout_seconds=value)


@pytest.mark.parametrize('argv',[[],['--mcap','unused'],['--bbox-source','bevfusion']])
def test_await_flag_cannot_enable_live_or_unbound_replay(monkeypatch,argv):
    from src.deployment import ros_frame_adapter as module
    monkeypatch.setattr('sys.argv',['adapter','--mcap-await-detections',*argv])
    with pytest.raises(SystemExit)as error:module.main()
    assert error.value.code==2
