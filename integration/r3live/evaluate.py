"""Read the actual upstream RGB PCD, verify nonempty color, and plot it."""
from pathlib import Path
import hashlib
import numpy as np


def evaluate(output):
    path=Path(output)/'rgb_pt.pcd'
    with path.open('rb') as stream:
        header={}
        while True:
            line=stream.readline().decode('ascii').strip()
            if not line: raise ValueError('Invalid PCD header')
            if line.startswith('#'): continue
            key,*values=line.split(); header[key]=values
            if key=='DATA': break
        if header['DATA']!=['binary']: raise ValueError('Expected original binary PCD')
        kinds={'F':'f','U':'u','I':'i'}
        dtype=np.dtype([(n,'<'+kinds[t]+size) for n,t,size in zip(header['FIELDS'],header['TYPE'],header['SIZE'])])
        pts=np.frombuffer(stream.read(),dtype=dtype,count=int(header['POINTS'][0]))
    if 'rgb' not in dtype.names: raise ValueError('Map lacks real RGB field')
    xyz=np.column_stack([pts[n] for n in ('x','y','z')])
    rgb=pts['rgb'].view('<u4')
    color=np.column_stack([(rgb>>16)&255,(rgb>>8)&255,rgb&255]).astype(np.uint8)
    finite=np.isfinite(xyz).all(axis=1);xyz,color=xyz[finite],color[finite]
    if len(xyz)==0: raise ValueError('Empty saved RGB map')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # register projection on Ubuntu's Matplotlib
    fig=plt.figure(figsize=(16,5))
    ax=fig.add_subplot(1,3,1,projection='3d')
    step=max(1,len(xyz)//30000)
    ax.scatter(xyz[::step,0],xyz[::step,1],xyz[::step,2],c=color[::step]/255.,s=.5)
    ax.set(xlabel='X (m)',ylabel='Y (m)',zlabel='Z (m)');ax.view_init(elev=24,azim=-120)
    for i,axes in enumerate([(0,1),(0,2)]):
        ax=fig.add_subplot(1,3,i+2)
        ax.scatter(xyz[:,axes[0]],xyz[:,axes[1]],c=color/255.,s=1)
        ax.set_aspect('equal');ax.set_xlabel('X (m)');ax.set_ylabel('Y (m)' if i==0 else 'Z (m)');ax.grid(alpha=.2)
    fig.suptitle('Actual R3LIVE RGB points — no recoloring or post-filtering')
    fig.tight_layout(rect=[0,0,1,.91]);fig.savefig(str(Path(output)/'colored_map.png'),dpi=170);plt.close(fig)
    return dict(colored_points=len(xyz),unique_colors=len(np.unique(color,axis=0)),
                rgb_map_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                map_bounds_min=xyz.min(axis=0).tolist(),map_bounds_max=xyz.max(axis=0).tolist())
