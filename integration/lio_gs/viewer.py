"""Interactive full-SH CUDA rendering with per-client OpenCV camera poses."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
import torch
import viser
from viser.transforms import SO3
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))
from integration.lio_gs.rendering import render


def run(args):
    torch.set_num_threads(2)
    torch.cuda.set_per_process_memory_fraction(.25)
    root=Path(args.capture).resolve()
    folders={'细节版':root}
    if args.baseline:folders['原版对照']=Path(args.baseline).resolve()
    models={}
    def load(label):
        if label not in models:
            checkpoint=torch.load(folders[label]/'gsplat/model.pt',map_location='cpu',weights_only=False)
            models[label]=({n:v.cuda() for n,v in checkpoint['params'].items()},checkpoint.get('eps2d',.3))
        return models[label]
    params,_=load('细节版')
    frames=json.loads((root/'dataset/frames.json').read_text())
    poses=np.array([r['c2w'] for r in frames]);centers=poses[:,:3,3]
    means=params['means'].cpu().numpy();lower,upper=np.quantile(means,[.01,.99],axis=0)
    middle=(lower+upper)/2;middle[2]=lower[2]+.95
    direction=middle-centers;distance=np.linalg.norm(direction[:,:2],axis=1)
    facing=np.sum(poses[:,:3,2]*direction,axis=1)/np.maximum(np.linalg.norm(direction,axis=1),1e-8)
    score=-distance+2*(1-facing)+.8*np.abs(centers[:,2]-middle[2])
    score[(facing<.7)|(centers[:,2]<lower[2]+.55)|(centers[:,2]>lower[2]+1.5)]=np.inf
    initial=int(np.argmin(score)) if np.isfinite(score).any() else 0
    camera=json.loads((root/'camera_input.json').read_text());original_k=np.array(camera['K']).reshape(3,3)
    server=viser.ViserServer(host='127.0.0.1',port=args.port)
    server.scene.set_up_direction('+z');server.scene.set_global_visibility(False)
    server.gui.add_markdown('### 室内 3DGS\n完整 SH 外观。拖动旋转、滚轮移动；切换版本可在同一视角对照。\n原始照片为 640×480，近距离细节受采集分辨率限制。')
    model=server.gui.add_dropdown('地图版本',options=list(folders),initial_value='细节版')
    toggle=server.gui.add_button('同视角切换版本',disabled=len(folders)<2)
    @toggle.on_click
    def switch(event):model.value='原版对照' if model.value=='细节版' else '细节版'
    quality=server.gui.add_slider('渲染宽度',min=640,max=1600,step=160,initial_value=1280)
    view=server.gui.add_slider('采集视点',min=0,max=len(frames)-1,step=1,initial_value=initial)
    button=server.gui.add_button('跳到所选视点')
    def position(client):
        pose=poses[view.value]
        with client.atomic():
            client.camera.position=pose[:3,3]
            client.camera.look_at=pose[:3,3]+pose[:3,2]*2
            client.camera.up_direction=-pose[:3,1]
            client.camera.fov=2*math.atan(camera['height']/(2*original_k[1,1]))
    @server.on_client_connect
    def connect(client):position(client)
    @button.on_click
    def reset(event):
        for client in server.get_clients().values():position(client)
    previous={};rendered=0;stats_path=root/'gsplat/full_sh_viewer.json'
    print('完整 SH 查看器：http://127.0.0.1:%d'%args.port,flush=True)
    while True:
        for identifier,client in server.get_clients().items():
            try:
                quaternion=np.array(client.camera.wxyz);translation=np.array(client.camera.position)
                aspect=float(client.camera.aspect);fov=float(client.camera.fov)
            except (AssertionError,KeyError):continue
            if not 0.2<aspect<6:continue
            width=int(quality.value);height=round(width/aspect)
            if height<1:continue
            signature=(model.value,width,height,fov,*quaternion,*translation)
            if previous.get(identifier)==signature:continue
            params,eps=load(model.value)
            pose=np.eye(4,dtype=np.float32)
            # viser camera WXYZ already maps OpenCV (+X right,+Y down,+Z forward)
            # to world. No additional y/z flip is appropriate here.
            pose[:3,:3]=SO3(quaternion).as_matrix();pose[:3,3]=translation
            focal=height/(2*math.tan(fov/2))
            k=np.array([[focal,0,width/2],[0,focal,height/2],[0,0,1]],np.float32)
            started=time.monotonic()
            with torch.no_grad():
                rgb,_,_=render(params,torch.tensor(pose,device='cuda'),torch.tensor(k,device='cuda'),width,height,eps2d=eps)
                image=(rgb.clamp(0,1).cpu().numpy()*255).astype(np.uint8)
            client.scene.set_background_image(image,format='png')
            previous[identifier]=signature;rendered+=1
            stats=dict(full_sh=True,sh_degree=3,transport='lossless PNG',frames_rendered=rendered,
                model=model.value,width=width,height=height,eps2d=eps,render_seconds=time.monotonic()-started,
                c2w=pose.tolist(),K=k.tolist(),url='http://127.0.0.1:%d'%args.port)
            temporary=stats_path.with_suffix('.tmp');temporary.write_text(json.dumps(stats,indent=2,ensure_ascii=False)+'\n');temporary.replace(stats_path)
        time.sleep(.15)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('capture');p.add_argument('--baseline')
    p.add_argument('--port',type=int,default=8891);run(p.parse_args())
