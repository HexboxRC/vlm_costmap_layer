"""Visible ground-boundary projection for a calibrated, rectified pinhole camera.

Pixel-error bounds express a caller-specified sensitivity assumption, not
calibrated VLM confidence. No lidar/background depth or oracle lookup occurs.
"""
from dataclasses import dataclass
import math
import numpy as np


class UnavailableGeometry(ValueError):
    """Observation must not generate geometry, including when it is ambiguous."""


@dataclass(frozen=True)
class Camera:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self):
        if (type(self.width) is not int or type(self.height) is not int or
                self.width < 2 or self.height < 2 or
                not all(math.isfinite(v) for v in (self.fx,self.fy,self.cx,self.cy)) or
                self.fx <= 0 or self.fy <= 0):
            raise ValueError('Invalid rectified camera calibration')


@dataclass(frozen=True)
class GroundPoint:
    x: float
    y: float
    sensitivity_m: float


class GroundProjector:
    def __init__(self, camera, rotation_map_from_optical, camera_position,
                 *, pixel_error_bound, max_sensitivity_m, max_range_m, floor_z=0.):
        self.camera = camera
        self.rotation = np.array(rotation_map_from_optical, dtype=float, copy=True)
        self.position = np.array(camera_position, dtype=float, copy=True)
        self.floor_z = float(floor_z)
        self.pixel_error = float(pixel_error_bound)
        self.max_sensitivity = float(max_sensitivity_m)
        self.max_range = float(max_range_m)
        if (self.rotation.shape != (3,3) or self.position.shape != (3,) or
                not np.isfinite(self.rotation).all() or not np.isfinite(self.position).all() or
                not np.allclose(self.rotation.T@self.rotation,np.eye(3),atol=1e-7,rtol=0) or
                not math.isclose(np.linalg.det(self.rotation),1.,abs_tol=1e-7)):
            raise ValueError('Expected a rigid camera-to-map transform')
        if (not all(math.isfinite(v) for v in (self.floor_z,self.pixel_error,
                                               self.max_sensitivity,self.max_range)) or
                self.pixel_error < 0 or self.max_sensitivity <= 0 or self.max_range <= 0 or
                self.position[2] <= self.floor_z):
            raise ValueError('Invalid projection limits or camera below floor')

    def _ray_intersection(self, u, v):
        k = self.camera
        ray = self.rotation @ np.array([(u-k.cx)/k.fx,(v-k.cy)/k.fy,1.])
        if ray[2] >= -1e-8:
            raise UnavailableGeometry('Ray is at/above the floor horizon')
        scale = (self.floor_z-self.position[2])/ray[2]
        point = self.position + scale*ray
        if math.hypot(*(point-self.position)[:2]) > self.max_range:
            raise UnavailableGeometry('Projected point is beyond accepted range')
        return point[:2]

    def point(self, u, v):
        k = self.camera
        if (not math.isfinite(u) or not math.isfinite(v) or
                not 0 <= u <= k.width-1 or not 0 <= v <= k.height-1):
            raise UnavailableGeometry('Pixel outside image')
        point = self._ray_intersection(u,v)
        # Do not clip the uncertainty neighborhood: image-edge evidence is incomplete.
        e = self.pixel_error
        if u-e < 0 or u+e > k.width-1 or v-e < 0 or v+e > k.height-1:
            raise UnavailableGeometry('Pixel uncertainty crosses the image boundary')
        sensitivity = max(float(np.linalg.norm(self._ray_intersection(u+dx,v+dy)-point))
                          for dx in (-e,e) for dy in (-e,e))
        if sensitivity > self.max_sensitivity:
            raise UnavailableGeometry('Pixel sensitivity exceeds accepted limit')
        return GroundPoint(float(point[0]),float(point[1]),sensitivity)

    def visible_base(self, endpoints, *, base_visible):
        if base_visible is not True or len(endpoints) != 2:
            raise UnavailableGeometry('Two visible base endpoints are required')
        a,b = [self.point(*p) for p in endpoints]
        if math.hypot(a.x-b.x,a.y-b.y) < 1e-6:
            raise UnavailableGeometry('Degenerate panel base')
        return a,b

    def ground_polygon(self, pixels):
        if len(pixels)<3:
            raise UnavailableGeometry('Ground polygon needs at least three vertices')
        points=[self.point(*p) for p in pixels]
        coords=[(p.x,p.y) for p in points]
        area=sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(coords,coords[1:]+coords[:1]))/2
        if abs(area)<1e-6:
            raise UnavailableGeometry('Degenerate ground polygon')
        # Reject self-intersections; do not invent vertex order for model output.
        def orient(a,b,c):return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
        n=len(coords)
        for i in range(n):
            a,b=coords[i],coords[(i+1)%n]
            for j in range(i+2,n):
                if (j+1)%n==i:continue
                c,d=coords[j],coords[(j+1)%n]
                overlap=(max(min(a[0],b[0]),min(c[0],d[0])) <= min(max(a[0],b[0]),max(c[0],d[0])) and
                         max(min(a[1],b[1]),min(c[1],d[1])) <= min(max(a[1],b[1]),max(c[1],d[1])))
                if overlap and orient(a,b,c)*orient(a,b,d)<=0 and orient(c,d,a)*orient(c,d,b)<=0:
                    raise UnavailableGeometry('Self-intersecting ground polygon')
        return points
