"""Local interactive browser viewer of learned Gaussian covariance/opacity/SH0."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation
import viser


def run(args):
    folder=Path(args.capture);data=np.load(folder/'gsplat/gaussians.npz')
    means=data['means'];q=data['quats'];q=q/np.linalg.norm(q,axis=1,keepdims=True)
    r=Rotation.from_quat(q[:,[1,2,3,0]]).as_matrix()
    covariance=(r*np.exp(2*data['scales'])[:,None,:])@r.transpose(0,2,1)
    rgb=np.clip(.5+.28209479177387814*data['sh0'][:,0],0,1)
    opacity=1/(1+np.exp(-data['opacities']))
    frames=json.loads((folder/'dataset/frames.json').read_text())
    poses=np.array([row['c2w'] for row in frames]);centers=poses[:,:3,3]
    lower,upper=np.quantile(means,[.01,.99],axis=0)
    middle=(lower+upper)/2;middle[2]=lower[2]+.95
    direction=middle-centers;distance=np.linalg.norm(direction[:,:2],axis=1)
    facing=np.sum(poses[:,:3,2]*direction,axis=1)/np.maximum(np.linalg.norm(direction,axis=1),1e-8)
    score=-distance+2*(1-facing)+.8*np.abs(centers[:,2]-middle[2])
    score[(facing<.7)|(centers[:,2]<lower[2]+.55)|(centers[:,2]>lower[2]+1.5)]=np.inf
    initial=int(np.argmin(score)) if np.isfinite(score).any() else 0
    server=viser.ViserServer(host='127.0.0.1',port=args.port)
    server.scene.set_up_direction('+z')
    server.scene.add_gaussian_splats('/room',means,covariance,rgb,opacity[:,None])
    server.gui.add_markdown('### 室内 3DGS\n拖动旋转、滚轮移动。此交互预览使用学习后的 SH0 颜色；六向图片与漫游视频使用完整 SH 外观。')
    view=server.gui.add_slider('采集视点',min=0,max=len(frames)-1,step=1,initial_value=initial)
    button=server.gui.add_button('跳到所选视点')
    def position(client):
        pose=np.array(frames[view.value]['c2w']);client.camera.position=pose[:3,3]
        client.camera.look_at=pose[:3,3]+pose[:3,2]*2
        client.camera.up_direction=-pose[:3,1]
    @server.on_client_connect
    def connect(client):position(client)
    @button.on_click
    def reset(event):
        for client in server.get_clients().values():position(client)
    print('打开 http://127.0.0.1:%d；Ctrl-C 退出'%args.port,flush=True)
    while True:time.sleep(.5)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('capture');p.add_argument('--port',type=int,default=8889)
    run(p.parse_args())
