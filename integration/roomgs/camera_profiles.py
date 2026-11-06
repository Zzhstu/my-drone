"""Nominal optical profiles; no invented distortion or sensor output modes."""
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
IMX577_PROFILE = ROOT / 'simulation/camera_profiles/imx577_10p5mm.json'
G1_PROFILE = ROOT / 'simulation/camera_profiles/g1_720_rectified.json'


def projection(width, height, horizontal_fov_rad):
    if width < 1 or height < 1 or not 0 < horizontal_fov_rad < math.pi:
        raise ValueError('Invalid pinhole dimensions/FOV')
    focal = width / (2 * math.tan(horizontal_fov_rad / 2))
    return dict(width=width, height=height,
                K=[[focal, 0., width/2], [0., focal, height/2], [0., 0., 1.]],
                horizontal_fov_rad=horizontal_fov_rad,
                horizontal_fov_deg=math.degrees(horizontal_fov_rad),
                vertical_fov_deg=math.degrees(2*math.atan(height/(2*focal))),
                diagonal_fov_deg=math.degrees(2*math.atan(math.hypot(width,height)/(2*focal))))


def profile_projection(width=960, height=720, profile=IMX577_PROFILE):
    path=Path(profile);spec=json.loads(path.read_text())
    return dict(**projection(width,height,math.radians(spec['horizontal_fov_deg'])),
                profile_id=spec['id'],profile_path=str(path),
                profile_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                specification=spec,D=[0.,0.,0.,0.,0.],
                distortion_policy=spec.get('simulation_scope','Ideal pinhole simulation; physical distortion not simulated'))
