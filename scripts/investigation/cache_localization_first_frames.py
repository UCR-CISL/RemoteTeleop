"""Extract immutable first-frame inputs for offline localization comparisons."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import cv2
import numpy as np


def transform(path, key):
    return np.asarray(json.loads(Path(path).read_text())[key], dtype=float).reshape(4, 4)


def save_frame(root, record, image, arrays, labels):
    directory = root / record['run_id']
    directory.mkdir(parents=True, exist_ok=True)
    arrays['box_labels'] = np.asarray(labels, dtype=str)
    with (directory / 'frame.npz').open('wb') as stream:
        np.savez_compressed(stream, **arrays)
    if not cv2.imwrite(str(directory / 'camera.png'), image):
        raise RuntimeError('camera PNG write failed')
    record.update(npz_path=str((directory/'frame.npz').resolve()), camera_path=str((directory/'camera.png').resolve()),
                  point_count=len(arrays['points_lidar']), box_count=len(labels), box_labels=labels)
    (directory/'metadata.json').write_text(json.dumps(record, indent=2)+'\n')
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset', choices=('cooperscene','nuscenes'), required=True)
    parser.add_argument('--output-root', type=Path, required=True)
    parser.add_argument('--processed-root', type=Path)
    parser.add_argument('--nuscenes-localizations', type=Path)
    parser.add_argument('--nuscenes-map-root', type=Path)
    parser.add_argument('--cooperscene-root', type=Path, default=Path('/mnt/bcc-data/websites/data.ucr.edu-datasets-cooperscene/mcap'))
    parser.add_argument('--cooperscene-inherited', type=Path)
    parser.add_argument('--cooperscene-initial', type=Path)
    parser.add_argument('--cooperscene-reference', type=Path, default=Path('config/cooperscene_riverside_iowa_enu_prior.json'))
    parser.add_argument('--cooperscene-map', type=Path)
    args = parser.parse_args()
    records = []
    if args.dataset == 'nuscenes':
        from src.data.processed_nuscenes import ProcessedNuScenesSequence
        for scene in ('152','164','171','200','209','359','529','916'):
            sequence = ProcessedNuScenesSequence(args.processed_root/scene, expected_frame_count=None)
            frame = sequence[0]
            reference = np.asarray(sequence.map_T_global)
            inv = np.linalg.inv(reference)
            boxes = frame.vehicles  # Includes pedestrians, not just proxy-supported vehicles.
            record = dict(run_id=f'nuscenes-{scene}', dataset='nuscenes', scene=scene, agent='1', frame_id=frame.frame_id,
                          sequence=0, timestamp_us=frame.timestamp_us, map_path=str(args.nuscenes_map_root/f'nuscenes-{scene}.ply'),
                          reference_kind='recorded_pose_alignment_baseline', source_path=str(args.processed_root/scene),
                          inherited_fit_frames=['100'], first_frame_only=True)
            arrays = dict(points_lidar=frame.lidar_points, dataset_T_lidar=inv@frame.map_T_lidar,
                          dataset_T_camera=inv@frame.map_T_camera, camera_intrinsic=frame.camera_intrinsic,
                          camera_distortion=frame.distortion,
                          inherited_map_T_dataset=transform(args.nuscenes_localizations/scene/'localization.json','gaussian_T_nuscenes_global'),
                          reference_map_T_dataset=reference, pipeline_initial_map_T_dataset=reference,
                          boxes_T_dataset_object=np.asarray([inv@b.map_T_object for b in boxes]).reshape(-1,4,4),
                          boxes_dimensions_lwh=np.asarray([b.dimensions_lwh for b in boxes]).reshape(-1,3))
            records.append(save_frame(args.output_root,record,frame.image,arrays,[b.category for b in boxes]))
            print(record['run_id'],record['frame_id'],record['point_count'],flush=True)
    else:
        from src.data.foxglove_mcap import FoxgloveMcapReader
        inherited=transform(args.cooperscene_inherited,'gaussian_T_cooperscene')
        initial=transform(args.cooperscene_initial,'gaussian_T_cooperscene')
        reference=transform(args.cooperscene_reference,'gaussian_T_cooperscene')
        for take in range(1,6):
            for agent in ('1','2','3'):
                path=args.cooperscene_root/f'{take}.mcap'
                frames=iter(FoxgloveMcapReader(path,agent=agent,include_lidar=True))
                frame=next(frames);frames.close()
                record=dict(run_id=f'cooperscene-{take}-{agent}',dataset='cooperscene',scene=str(take),agent=agent,
                            frame_id=frame.metadata['frame_id'], sequence=0,timestamp_us=0,source_timestamp_ns=frame.timestamp_ns,
                            map_path=str(args.cooperscene_map),reference_kind='shared_enu_prior',source_path=str(path),
                            inherited_fit_frames=['take1/agent1/481260'], first_frame_only=True)
                arrays=dict(points_lidar=frame.lidar_points,dataset_T_lidar=np.asarray(frame.metadata['map_T_lidar']).reshape(4,4),
                            dataset_T_camera=np.asarray(frame.metadata['world_T_camera']).reshape(4,4),
                            camera_intrinsic=np.asarray(frame.metadata['camera_intrinsic']).reshape(3,3),
                            camera_distortion=np.asarray(frame.metadata.get('camera_distortion',[])),
                            inherited_map_T_dataset=inherited,reference_map_T_dataset=reference,pipeline_initial_map_T_dataset=initial,
                            boxes_T_dataset_object=np.asarray([v.world_T_object for v in frame.vehicles]).reshape(-1,4,4),
                            boxes_dimensions_lwh=np.asarray([v.dimensions_lwh for v in frame.vehicles]).reshape(-1,3))
                records.append(save_frame(args.output_root,record,frame.image_bgr,arrays,[v.text_label for v in frame.vehicles]))
                print(record['run_id'],record['frame_id'],record['point_count'],flush=True)
    (args.output_root/f'{args.dataset}-manifest.json').write_text(json.dumps(records,indent=2)+'\n')

if __name__=='__main__':
    main()
