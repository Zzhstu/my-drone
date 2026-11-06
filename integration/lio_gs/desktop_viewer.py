"""Native GLFW/OpenGL display of full-SH gsplat CUDA rendering, with no HTTP."""
import argparse
import json
import math
from pathlib import Path
import sys
import time
import numpy as np
from PIL import Image
import torch
import torch.nn.functional as F
import glfw
from OpenGL import GL as gl
from gsplat import rasterization
ROOT=Path(__file__).resolve().parents[2];sys.path.insert(0,str(ROOT))


def load_parameters(path):
    if path.suffix=='.pt':
        model=torch.load(path,map_location='cpu',weights_only=False)
        return model['params'],model.get('eps2d',.3)
    if path.suffix!='.ply':raise ValueError('Use a trained Gaussian .pt or .ply')
    from plyfile import PlyData
    p=PlyData.read(path)['vertex'].data
    values=dict(means=np.column_stack([p[n] for n in ['x','y','z']]),
        scales=np.column_stack([p['scale_%d'%i] for i in range(3)]),
        quats=np.column_stack([p['rot_%d'%i] for i in range(4)]),opacities=p['opacity'],
        sh0=np.column_stack([p['f_dc_%d'%i] for i in range(3)])[:,None,:],
        shN=np.column_stack([p['f_rest_%d'%i] for i in range(45)]).reshape(-1,3,15).transpose(0,2,1))
    return {n:torch.from_numpy(np.array(v,copy=True)) for n,v in values.items()},.1


class Camera:
    def __init__(self,pose):self.set_pose(pose)
    def set_pose(self,pose):
        self.pose=np.array(pose,dtype=np.float32,copy=True)
        f=self.pose[:3,2];self.yaw=math.atan2(f[1],f[0]);self.pitch=math.asin(np.clip(f[2],-1,1))
    def turn(self,dx,dy):
        self.yaw-=dx*.003;self.pitch=np.clip(self.pitch-dy*.003,-1.52,1.52)
        f=np.array([math.cos(self.pitch)*math.cos(self.yaw),math.cos(self.pitch)*math.sin(self.yaw),math.sin(self.pitch)])
        right=np.cross(f,[0,0,1]);right/=np.linalg.norm(right)
        self.pose[:3,:3]=np.column_stack([right,np.cross(f,right),f])
    def move(self,right=0,forward=0,up=0):
        self.pose[:3,3]+=self.pose[:3,0]*right+self.pose[:3,2]*forward+np.array([0,0,up])


def run(args):
    root=Path(args.capture).resolve();path=Path(args.model).resolve() if args.model else root/'gsplat/model.pt'
    torch.set_num_threads(2);torch.cuda.set_per_process_memory_fraction(.25)
    params,eps=load_parameters(path)
    if args.eps2d is not None:eps=args.eps2d
    params={n:v.to(device='cuda',dtype=torch.float32) for n,v in params.items()}
    reference_params=params if args.self_test else None
    prepared=dict(means=params['means'],quats=F.normalize(params['quats'],dim=-1),scales=params['scales'].exp(),
        opacities=params['opacities'].sigmoid(),colors=torch.cat([params['sh0'],params['shN']],dim=1))
    del params
    frames=json.loads((root/'dataset/frames.json').read_text());poses=np.array([r['c2w'] for r in frames],np.float32)
    # The same measured starting viewpoint used by the validated browser viewer.
    means=prepared['means'].cpu().numpy();lower,upper=np.quantile(means,[.01,.99],axis=0)
    middle=(lower+upper)/2;middle[2]=lower[2]+.95;centers=poses[:,:3,3];direction=middle-centers
    facing=np.sum(poses[:,:3,2]*direction,axis=1)/np.maximum(np.linalg.norm(direction,axis=1),1e-8)
    score=-np.linalg.norm(direction[:,:2],axis=1)+2*(1-facing)+.8*np.abs(centers[:,2]-middle[2])
    score[(facing<.7)|(centers[:,2]<lower[2]+.55)|(centers[:,2]>lower[2]+1.5)]=np.inf
    initial=int(np.argmin(score)) if np.isfinite(score).any() else 0
    settings_path=root/'viewer_settings.json'
    if settings_path.exists():
        initial=int(json.loads(settings_path.read_text())['initial_frame'])
        if not 0<=initial<len(poses):raise ValueError('Viewer initial frame outside captured dataset')
    camera=Camera(poses[initial]);info=json.loads((root/'camera_input.json').read_text())
    native_k=np.array(info['K']).reshape(3,3);fov=2*math.atan(info['height']/(2*native_k[1,1]))
    output=Path(args.report_dir).resolve() if args.report_dir else ROOT/'evaluation/lio_gsplat_desktop_20261103'
    output.mkdir(parents=True,exist_ok=True)
    if not glfw.init():raise RuntimeError('Cannot initialize GLFW; a local graphical desktop is required')
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR,2);glfw.window_hint(glfw.CONTEXT_VERSION_MINOR,1)
    window=glfw.create_window(args.width,args.height,args.title+' - Full SH3',None,None)
    if not window:glfw.terminate();raise RuntimeError('Cannot create native OpenGL window')
    glfw.make_context_current(window);glfw.swap_interval(0 if args.self_test else 1)
    renderer=gl.glGetString(gl.GL_RENDERER).decode();vendor=gl.glGetString(gl.GL_VENDOR).decode()
    if 'llvmpipe' in renderer.lower() or 'softpipe' in renderer.lower():raise RuntimeError('Software OpenGL renderer selected: '+renderer)
    texture=gl.glGenTextures(1);gl.glBindTexture(gl.GL_TEXTURE_2D,texture)
    gl.glTexParameteri(gl.GL_TEXTURE_2D,gl.GL_TEXTURE_MIN_FILTER,gl.GL_LINEAR)
    gl.glTexParameteri(gl.GL_TEXTURE_2D,gl.GL_TEXTURE_MAG_FILTER,gl.GL_LINEAR)
    gl.glPixelStorei(gl.GL_UNPACK_ALIGNMENT,1);gl.glPixelStorei(gl.GL_PACK_ALIGNMENT,1)
    gl.glDisable(gl.GL_DEPTH_TEST);gl.glDisable(gl.GL_LIGHTING);gl.glEnable(gl.GL_TEXTURE_2D)
    state=dict(dirty=True,last_mouse=None,speed=.8,quality=True,index=initial,moving_until=0.,key_generation=0,last_key=None)
    def key_callback(w,key,scan,action,mods):
        if action!=glfw.PRESS:return
        state['key_generation']+=1;state['last_key']=key
        if key==glfw.KEY_ESCAPE:glfw.set_window_should_close(window,True)
        if key==glfw.KEY_HOME:camera.set_pose(poses[initial]);state['index']=initial
        if key in [glfw.KEY_LEFT,glfw.KEY_RIGHT]:
            state['index']=int(np.clip(state['index']+(-40 if key==glfw.KEY_LEFT else 40),0,len(poses)-1));camera.set_pose(poses[state['index']])
        if key==glfw.KEY_F:state['quality']=not state['quality']
        if key in [glfw.KEY_EQUAL,glfw.KEY_KP_ADD]:state['speed']=min(8.,state['speed']*1.5)
        if key in [glfw.KEY_MINUS,glfw.KEY_KP_SUBTRACT]:state['speed']=max(.03,state['speed']/1.5)
        state['dirty']=True
    def mouse_callback(w,x,y):
        old=state['last_mouse'];state['last_mouse']=(x,y)
        if old is None:return
        dx,dy=x-old[0],y-old[1]
        if glfw.get_mouse_button(window,glfw.MOUSE_BUTTON_LEFT)==glfw.PRESS:camera.turn(dx,dy)
        elif glfw.get_mouse_button(window,glfw.MOUSE_BUTTON_RIGHT)==glfw.PRESS:camera.move(right=-dx*.002,up=dy*.002)
        else:return
        state['dirty']=True;state['moving_until']=time.monotonic()+.18
    def scroll_callback(w,x,y):camera.move(forward=y*.15);state.update(dirty=True,moving_until=time.monotonic()+.18)
    glfw.set_key_callback(window,key_callback);glfw.set_cursor_pos_callback(window,mouse_callback);glfw.set_scroll_callback(window,scroll_callback)
    glfw.set_framebuffer_size_callback(window,lambda *a:state.update(dirty=True))
    print('Native OpenGL:',vendor,renderer,flush=True)
    print('WASD move; Q/E down/up; Shift faster; left-drag look; right-drag pan; wheel forward/back; Home reset; arrows captured views; F quality; Esc close',flush=True)
    frames_rendered=0;frame_times=[];render_times=[];tex_size=None;start=time.monotonic();previous=start;last_signature=None
    first_pose=None;first_picture=None;final_picture=None;gl_difference=None
    reference_difference=None;reference_pixels=None
    last_live=0.;last_live_key=-1;width=height=0
    try:
        while not glfw.window_should_close(window):
            now=time.monotonic();dt=min(.05,now-previous);previous=now;glfw.poll_events()
            speed=state['speed']*dt*(3 if glfw.get_key(window,glfw.KEY_LEFT_SHIFT)==glfw.PRESS else 1)
            lr=int(glfw.get_key(window,glfw.KEY_D)==glfw.PRESS)-int(glfw.get_key(window,glfw.KEY_A)==glfw.PRESS)
            fb=int(glfw.get_key(window,glfw.KEY_W)==glfw.PRESS)-int(glfw.get_key(window,glfw.KEY_S)==glfw.PRESS)
            ud=int(glfw.get_key(window,glfw.KEY_E)==glfw.PRESS)-int(glfw.get_key(window,glfw.KEY_Q)==glfw.PRESS)
            if lr or fb or ud:camera.move(lr*speed,fb*speed,ud*speed);state.update(dirty=True,moving_until=now+.18)
            if args.self_test and frames_rendered>0:
                camera.turn(.12,0);state['dirty']=True
            ww,hh=glfw.get_framebuffer_size(window)
            if ww<1 or hh<1:glfw.wait_events_timeout(.1);continue
            width=min(ww,args.render_width if state['quality'] else 960);height=round(hh*width/ww)
            signature=(width,height,*camera.pose.ravel())
            if not state['dirty'] and signature==last_signature:glfw.wait_events_timeout(.025);continue
            begun=time.monotonic();focal=height/(2*math.tan(fov/2))
            k=torch.tensor([[focal,0,width/2],[0,focal,height/2],[0,0,1]],device='cuda',dtype=torch.float32)[None]
            # Match training's float32 inverse exactly. At warehouse-scale
            # translations, the transpose shortcut can shift edge pixels.
            view=torch.linalg.inv(torch.tensor(camera.pose,device='cuda'))[None]
            with torch.no_grad():
                rgb,_,_=rasterization(**prepared,viewmats=view,Ks=k,
                    width=width,height=height,near_plane=.15,far_plane=30.,sh_degree=3,packed=False,eps2d=eps)
                picture=(rgb[0].clamp(0,1)*255).to(torch.uint8).contiguous().cpu().numpy()
                if args.self_test and frames_rendered==0:
                    from integration.lio_gs.rendering import render as reference_render
                    reference,_,_=reference_render(reference_params,torch.tensor(camera.pose,device='cuda'),k[0],width,height,eps2d=eps)
                    reference_difference=float((reference-rgb[0]).abs().max())
                    reference_image=(reference.clamp(0,1)*255).to(torch.uint8).cpu().numpy()
                    reference_pixels=int(np.abs(reference_image.astype(int)-picture.astype(int)).max())
                    reference_params=None
            rendered=time.monotonic();gl.glViewport(0,0,ww,hh)
            gl.glMatrixMode(gl.GL_PROJECTION);gl.glLoadIdentity();gl.glOrtho(0,1,1,0,-1,1)
            gl.glMatrixMode(gl.GL_MODELVIEW);gl.glLoadIdentity();gl.glBindTexture(gl.GL_TEXTURE_2D,texture)
            if tex_size!=(width,height):
                gl.glTexImage2D(gl.GL_TEXTURE_2D,0,gl.GL_RGB8,width,height,0,gl.GL_RGB,gl.GL_UNSIGNED_BYTE,picture);tex_size=(width,height)
            else:gl.glTexSubImage2D(gl.GL_TEXTURE_2D,0,0,0,width,height,gl.GL_RGB,gl.GL_UNSIGNED_BYTE,picture)
            gl.glBegin(gl.GL_QUADS)
            for u,v in [(0,0),(1,0),(1,1),(0,1)]:gl.glTexCoord2f(u,v);gl.glVertex2f(u,v)
            gl.glEnd()
            if frames_rendered==0:
                gl.glFinish();raw=gl.glReadPixels(0,0,ww,hh,gl.GL_RGB,gl.GL_UNSIGNED_BYTE)
                first_picture=np.frombuffer(raw,np.uint8).reshape(hh,ww,3)[::-1].copy();first_pose=camera.pose.copy()
                Image.fromarray(first_picture).save(output/'native_initial.png')
                if (ww,hh)==(width,height):gl_difference=int(np.abs(first_picture.astype(int)-picture.astype(int)).max())
            glfw.swap_buffers(window);elapsed=time.monotonic()-begun;frames_rendered+=1
            if frames_rendered>10:frame_times.append(elapsed);render_times.append(rendered-begun)
            if frames_rendered%15==0 or frames_rendered==1:
                fps=('%.1f FPS'%(1/np.median(frame_times[-120:]))) if frame_times else 'Ready'
                glfw.set_window_title(window,args.title+' | Full SH3 | %s | WASD move, L-drag look, wheel, Home reset, F quality'%fps)
            if now-last_live>.25 or last_live_key!=state['key_generation']:
                live=dict(c2w=camera.pose.tolist(),frames_rendered=frames_rendered,last_key=state['last_key'],
                    key_generation=state['key_generation'],render_resolution=[width,height],window_size=[ww,hh],
                    model=str(path),gl_renderer=renderer,median_frame_fps=1/float(np.median(frame_times[-120:])) if frame_times else None)
                temporary=output/'native_live.tmp';temporary.write_text(json.dumps(live,indent=2)+'\n');temporary.replace(output/'native_live.json')
                last_live=now;last_live_key=state['key_generation']
            state['dirty']=False;last_signature=signature
            if args.self_test and frames_rendered>=args.test_frames:
                gl.glReadBuffer(gl.GL_FRONT);raw=gl.glReadPixels(0,0,ww,hh,gl.GL_RGB,gl.GL_UNSIGNED_BYTE)
                final_picture=np.frombuffer(raw,np.uint8).reshape(hh,ww,3)[::-1].copy();Image.fromarray(final_picture).save(output/'native_moved.png');break
    finally:
        glfw.destroy_window(window);glfw.terminate()
    report=dict(native_desktop=True,http_used=False,gaussians=len(prepared['means']),model=str(path),eps2d=eps,sh_degree=3,
        gl_vendor=vendor,gl_renderer=renderer,cuda_gpu=torch.cuda.get_device_name(0),frames_rendered=frames_rendered,
        render_resolution=[width,height],median_render_ms=1000*float(np.median(render_times)) if render_times else None,
        median_frame_ms=1000*float(np.median(frame_times)) if frame_times else None,
        median_frame_fps=1/float(np.median(frame_times)) if frame_times else None,
        gl_display_max_pixel_difference=gl_difference,first_c2w=first_pose.tolist() if first_pose is not None else None,
        reference_render_max_float_difference=reference_difference,reference_render_max_pixel_difference=reference_pixels,
        motion_pixel_difference=float(np.abs(first_picture.astype(float)-final_picture.astype(float)).mean()) if final_picture is not None else None,
        camera_changed=bool(first_pose is not None and np.max(np.abs(first_pose-camera.pose))>1e-5),
        gpu_peak_mib=torch.cuda.max_memory_allocated()/2**20,
        scope='Actual native X11/OpenGL/CUDA loop; self-test camera motion is automated, not physical mouse validation; timing includes GPU readback and GL swap, excludes startup/idle',
        self_test=args.self_test,wall_seconds=time.monotonic()-start)
    (output/'native_validation.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2),flush=True)
    if args.self_test and (not report['camera_changed'] or report['motion_pixel_difference']<.5 or gl_difference not in (None,0,1) or reference_pixels>1):
        raise RuntimeError('Native rendering/camera/display validation failed')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('capture');p.add_argument('--model');p.add_argument('--eps2d',type=float)
    p.add_argument('--width',type=int,default=1280);p.add_argument('--height',type=int,default=800)
    p.add_argument('--render-width',type=int,default=1280);p.add_argument('--self-test',action='store_true')
    p.add_argument('--title',default='AstraDrone 3DGS Desktop')
    p.add_argument('--test-frames',type=int,default=160);p.add_argument('--report-dir');run(p.parse_args())
