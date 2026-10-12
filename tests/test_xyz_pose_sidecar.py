import copy
import hashlib
import json
from types import SimpleNamespace

import numpy as np
import pytest

from src.data.xyz_pose_sidecar import VerifiedXYZPoseSidecar
from src.data.cooperscene_mcap import _serialize_image, _serialize_metadata
from src.deployment.ros_frame_adapter import McapFrameReplay, RosFrameAdapter, RosFrameAdapterConfig, RosAnalysisAssembler
from src.deployment.vehicle import VehicleAnalysisSpool, VehicleFrameSpool
from src.realtime.protocol import FrameDetections


@pytest.fixture
def fixture(tmp_path):
    image = np.array([[[1, 2, 3]]], dtype=np.uint8)
    points = np.array([[2., 0., .4, .8]], dtype=np.float32)
    payload = tmp_path / 'points.npz'
    np.savez(payload, lidar=points)
    path = tmp_path / 'recording.mcap'
    path.write_bytes(b'bounded-reader-fixture')
    K = np.eye(3)
    pose = np.eye(4)
    pose[2, 3] = 1.234
    frames = [dict(source_index=i, frame_id=str(100+i), timestamp_us=1_000_000+(i-7)*100_003,
        lidar_timestamp_ns=(1_000_000+(i-7)*100_003)*1000+123, image_timestamp_ns=999_000_000+(i-7)*100_000_000,
        recorded_timestamp_us=i*100_000, world_T_lidar=pose.tolist(),
        image_bgr_sha256=hashlib.sha256(image.tobytes()).hexdigest(),
        lidar_f32_sha256=hashlib.sha256(points.tobytes()).hexdigest(),
        points_npz=dict(path=payload.name,sha256=hashlib.sha256(payload.read_bytes()).hexdigest()))for i in (7,8)]
    manifest=dict(version=1,take=1,agent='2',mcap=dict(name=path.name,size=path.stat().st_size,mtime_ns=path.stat().st_mtime_ns),
        provenance=dict(recipe_sha256='a'*64,map_sha256='b'*64,source_inputs_sha256='c'*64,policy='verified archived XYZ'),
        calibration=dict(image_width=1,image_height=1,camera_intrinsic=K.tolist(),camera_T_lidar=np.eye(4).tolist()),frames=frames)
    sidecar=tmp_path/'poses.json'
    sidecar.write_text(json.dumps(manifest))
    metadata=dict(version=1,scene_generation='scene',frame_id='107',timestamp_us=700_000,
        camera_intrinsic=K.ravel().tolist(),world_T_camera=np.eye(4).ravel().tolist(),world_T_ego=np.eye(4).ravel().tolist(),
        boxes=[{'arbitrary_GT':True}],reference_world_T_lidar=np.eye(4).tolist())
    return path,sidecar,manifest,metadata,image,points


def test_binding_preserves_xyz_and_capture_clock_excludes_gt_and_supplements(fixture):
    path,p,m,meta,image,points=fixture
    s=VerifiedXYZPoseSidecar(p);s.validate_mcap(path,'2')
    result,actual=s.bind(7,meta,image,None)
    assert result['timestamp_us']==1_000_000 and result['recorded_timestamp_us']==700_000
    assert result['request_id']=='scene:107' and not result['end_of_scene']
    assert not any(k in result for k in ('boxes','gt','reference_world_T_lidar'))
    assert np.array_equal(actual,points)
    assert np.array_equal(np.asarray(result['map_T_lidar']).reshape(4,4),m['frames'][0]['world_T_lidar'])
    assert result['world_T_ego']==result['map_T_lidar']==result['world_T_camera']


@pytest.mark.parametrize('bad',['frame','index','clock','K','C','RGB','points'])
def test_mismatched_actual_sensor_identity_rejected(fixture,bad):
    _,p,_,meta,image,points=fixture
    meta=copy.deepcopy(meta);index=7
    if bad=='frame':meta['frame_id']='wrong'
    if bad=='index':index=8
    if bad=='clock':meta['timestamp_us']+=1
    if bad=='K':meta['camera_intrinsic'][0]=2.
    if bad=='C':meta['world_T_camera'][3]=.1
    if bad=='RGB':image=image.copy();image[0,0,0]=5
    if bad=='points':points=points.copy();points[0,0]+=1.
    with pytest.raises(ValueError):VerifiedXYZPoseSidecar(p).bind(index,meta,image,points)


@pytest.mark.parametrize('bad',['duplicate','rigid','image_clock','take','GT','NPZ'])
def test_invalid_sidecar_or_supplement_rejected(fixture,bad):
    _,p,m,meta,image,_=fixture
    if bad=='duplicate':m['frames'][1]['frame_id']=m['frames'][0]['frame_id']
    if bad=='rigid':m['frames'][0]['world_T_lidar'][0][0]=2.
    if bad=='image_clock':m['frames'][0]['image_timestamp_ns']=0
    if bad=='take':m['take']=True
    if bad=='GT':m['frames'][0]['gt']=[]
    if bad=='NPZ':m['frames'][0]['points_npz']['sha256']='d'*64
    p.write_text(json.dumps(m))
    with pytest.raises(ValueError):
        s=VerifiedXYZPoseSidecar(p);s.bind(7,meta,image,None)


def test_actual_replay_branch_nonzero_range_real_analysis_spool(fixture,tmp_path):
    path,p,m,meta,image,_=fixture
    records=[]
    for i in (7,8):
        clock=i*100_000
        d=FrameDetections(frame_id=str(100+i),request_id=f'old:{i}',scene_generation='scene',timestamp_us=clock,
            image_jpeg=b'x',camera_intrinsic=meta['camera_intrinsic'],world_T_camera=meta['world_T_camera'],
            world_T_ego=meta['world_T_ego'],boxes=())
        for topic,data in [('/image',_serialize_image(timestamp_ns=clock*1000,frame_id='cam',height=1,width=1,data=image.tobytes())),('/meta',_serialize_metadata(d))]:
            records.append((SimpleNamespace(name='sensor_msgs/msg/Image'if topic=='/image'else 'std_msgs/msg/String'),SimpleNamespace(topic=topic,message_encoding='cdr'),SimpleNamespace(data=data,sequence=i,log_time=clock*1000)))
    class Reader:
        def get_summary(self):return None
        def iter_messages(self,**kwargs):yield from records
    class Publisher:
        def send(self,*a,**kw):pass
    analysis=VehicleAnalysisSpool(tmp_path/'analysis');assembly=RosAnalysisAssembler(analysis,expected_samples=2)
    snapshots=VehicleFrameSpool(tmp_path/'snapshots');adapter=RosFrameAdapter(Publisher(),RosFrameAdapterConfig(),analysis_assembler=assembly,snapshot_sink=snapshots)
    replay=McapFrameReplay(adapter,image_topic='/image',metadata_topic='/meta',expected_samples=2,start_sequence=7,
        agent='2',xyz_pose_sidecar=VerifiedXYZPoseSidecar(p),reader_factory=lambda f:Reader(),sleep=lambda seconds:None,monotonic=lambda:0.)
    # No detector: one sleep per record gap, test pacing separately from original capture clocks.
    replay.replay(path);assembly.close()
    for i,clock in enumerate((1_000_000,1_100_003)):
        frame=analysis.get('scene',i);assert frame.timestamp_us==clock and frame.request_id==f'scene:{107+i}'
        assert frame.boxes==() and frame.end_of_scene==(i==1)
        assert np.asarray(frame.world_T_ego).reshape(4,4)[2,3]==1.234
        assert snapshots.get('scene',i).timestamp_us==clock
    assert analysis.end('scene') is not None
    with pytest.raises(ValueError,match='expected_samples'):
        McapFrameReplay(adapter,image_topic='/image',metadata_topic='/meta',expected_samples=3,agent='2',xyz_pose_sidecar=VerifiedXYZPoseSidecar(p))


def test_foxglove_sidecar_validates_contained_cloud_before_delivery(fixture,tmp_path,monkeypatch):
    path,p,m,meta,image,points=fixture
    from src.data import foxglove_mcap
    calls=[]
    class Frames:
        def __init__(self,*a,**kw):assert kw['include_lidar'] is True
        def __iter__(self):
            for i in (7,8):
                old=dict(meta,frame_id=str(100+i),timestamp_us=i*100_000)
                yield SimpleNamespace(sequence=i,timestamp_ns=i*100_000_000,metadata=old,image_bgr=image,lidar_points=points)
    monkeypatch.setattr(foxglove_mcap,'FoxgloveMcapReader',Frames)
    class Assembly:
        def receive_image(self,x):calls.append(('image',x.header.stamp.sec,x.header.stamp.nanosec))
        def raise_if_failed(self):pass
        def receive_metadata(self,payload,sequence):pass
    class Publisher:
        def send(self,*a,**kw):pass
    adapter=RosFrameAdapter(Publisher(),RosFrameAdapterConfig(),analysis_assembler=Assembly())
    replay=McapFrameReplay(adapter,image_topic='/image',metadata_topic='/meta',expected_samples=2,agent='2',
        xyz_pose_sidecar=VerifiedXYZPoseSidecar(p),sleep=lambda x:None,monotonic=lambda:0.)
    replay._replay_foxglove(path)
    assert calls==[('image',1,0),('image',1,100_003_000)]
    with pytest.raises(ValueError,match='extrinsic'):
        bad=dict(meta);bad['world_T_camera'][3]=.01
        VerifiedXYZPoseSidecar(p).bind(7,bad,image,points)


def test_cli_sidecar_cannot_start_live_or_self_scoring(tmp_path,monkeypatch):
    from src.deployment import ros_frame_adapter as module
    monkeypatch.setattr('sys.argv',['adapter','--xyz-pose-sidecar',str(tmp_path/'poses.json')])
    with pytest.raises(SystemExit) as error:module.main()
    assert error.value.code==2


@pytest.mark.parametrize('bad',['shape','dtype'])
def test_image_geometry_not_only_bytes_is_validated(fixture,bad):
    _,p,_,meta,image,points=fixture
    image=image.reshape(3,1,1) if bad=='shape' else image.astype(np.int8)
    assert image.tobytes()==fixture[4].tobytes()
    with pytest.raises(ValueError,match='shape/dtype'):VerifiedXYZPoseSidecar(p).bind(7,meta,image,points)
