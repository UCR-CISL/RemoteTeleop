"""Compare original and advanced-frame recorded cameras, GSplat renders and scans."""
import json
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scripts.investigation.first_frame_xyz import source_points
from src.localization.gaussian_map import load_gaussian_map
from src.viz.localization_visualizer import render_gaussian_camera_buffers


class FrameAdvanceVisuals:
    def __init__(self,root=Path('artifacts/frame-advance-localization')):
        self.root=root
        self.output=root/'visuals'
        self.output.mkdir(parents=True,exist_ok=True)
        self.map_path=None
        self.gaussian_map=None

    @staticmethod
    def label(image,text):
        canvas=cv2.copyMakeBorder(image,36,0,0,0,cv2.BORDER_CONSTANT,value=(245,245,245))
        cv2.putText(canvas,text,(7,23),cv2.FONT_HERSHEY_SIMPLEX,.45,(10,10,10),1,cv2.LINE_AA)
        return canvas

    def render(self,metadata_path,result_path,variant='baseline'):
        selected=json.loads(Path(metadata_path).read_text())
        result=json.loads(Path(result_path).read_text())
        original_id=f"nuscenes-{selected['scene']}" if selected['dataset']=='nuscenes' else f"cooperscene-{selected['scene']}-{selected['agent']}"
        first=Path('artifacts/first-frame-localization')
        original=json.loads((first/'inputs'/original_id/'metadata.json').read_text())
        original_result=json.loads((first/'xyz'/variant/f'{original_id}.json').read_text())
        if self.map_path!=selected['map_path']:
            self.gaussian_map=load_gaussian_map(selected['map_path'])
            self.map_path=selected['map_path']
        panels=[]
        figure,axes=plt.subplots(2,2,figsize=(13,10),layout='constrained')
        backends=[]
        qualifies=result['accepted'] and result['fresh_error']['xyz_m']<.1 and selected.get('sequence_fraction',0)<=.25
        selected_label='Chosen frame' if qualifies else 'Diagnostic frame'
        for column,(label,metadata,fit) in enumerate((('Original frame',original,original_result),(selected_label,selected,result))):
            frame=np.load(metadata['npz_path'])
            transform=np.asarray(fit['map_T_dataset'])
            status='accepted' if fit['accepted'] else 'REJECTED'
            seconds=metadata.get('offset_seconds',metadata.get('elapsed_seconds',0))
            percent=100*metadata.get('sequence_fraction',0)
            title=f"{label}: {seconds:.1f}s ({percent:.1f}%), XYZ {fit['fresh_error']['xyz_m']:.3f}m {status}"
            camera=cv2.imread(metadata['camera_path'])
            height,width=camera.shape[:2]
            distortion=frame['camera_distortion']
            if distortion.size and np.any(distortion):
                camera=cv2.undistort(camera,frame['camera_intrinsic'],distortion)
            camera=cv2.resize(camera,(width//4,height//4))
            buffers=render_gaussian_camera_buffers(self.gaussian_map,transform@frame['dataset_T_camera'],
                frame['camera_intrinsic'],(width,height),downsample=4,device='cuda')
            rendered=np.clip(buffers.rgb[...,::-1]*255,0,255).astype(np.uint8)
            overlay=cv2.addWeighted(camera,.5,rendered,.5,0)
            panels.append(np.hstack([self.label(camera,title),self.label(rendered,'GSplat at independently fitted pose'),
                                     self.label(overlay,'50% recorded / 50% GSplat')]))
            backends.append(buffers.backend)
            reference=frame['reference_map_T_dataset']
            center=frame['dataset_T_lidar'][:3,3]
            points=source_points(frame,mask_dynamic=True)
            points=((points@transform[:3,:3].T+transform[:3,3])-reference[:3,3])@reference[:3,:3]-center
            target=(self.gaussian_map.means-reference[:3,3])@reference[:3,:3]-center
            target=target[(np.abs(target[:,:2])<60).all(axis=1)&(np.abs(target[:,2])<20)]
            target=target[::max(1,len(target)//80000)]
            points=points[::max(1,len(points)//20000)]
            for row,dims in enumerate(((0,1),(0,2))):
                axis=axes[row,column]
                axis.scatter(target[:,dims[0]],target[:,dims[1]],s=.25,c='gray',alpha=.4)
                axis.scatter(points[:,dims[0]],points[:,dims[1]],s=.35,c='#d34f35',alpha=.5)
                axis.plot(0,0,'+',color='#2266dd',ms=10)
                axis.set(xlim=(-55,55),ylim=(-55,55) if row==0 else (-10,15),
                         xlabel='Dataset X from current reference LiDAR (m)',ylabel='Dataset Y (m)' if row==0 else 'Dataset Z (m)',
                         title=title if row==0 else 'Side view')
                axis.set_aspect('equal');axis.grid(alpha=.2)
        stem=f'{original_id}-{variant}'
        cv2.imwrite(str(self.output/f'{stem}-camera.png'),np.vstack(panels))
        figure.suptitle(f'{original_id}: original versus advanced frame ({variant})\nGray: map centers; orange: annotation-masked scan. Display crop only; errors are reference-relative.')
        figure.savefig(self.output/f'{stem}-cloud.png',dpi=130)
        plt.close(figure)
        (self.output/f'{stem}.json').write_text(json.dumps(dict(rendering_version=2,metadata_path=str(metadata_path),result_path=str(result_path),
            variant=variant,render_backends=backends),indent=2)+'\n')
        print(stem,flush=True)


if __name__=='__main__':
    root=Path('artifacts/frame-advance-localization')
    renderer=FrameAdvanceVisuals(root)
    for item in json.loads((root/'selections.json').read_text()):
        selected=item['selected_visual']
        if selected is None or selected['frame_offset']==0:
            continue
        manifest=renderer.output/f"{item['original_run_id']}-{item['variant']}.json"
        if manifest.exists():
            previous=json.loads(manifest.read_text())
            if previous.get('rendering_version')==2 and previous['result_path']==selected['result_path']:
                continue
        renderer.render(root/item['dataset']/'inputs'/selected['run_id']/'metadata.json',
                        selected['result_path'],item['variant'])
