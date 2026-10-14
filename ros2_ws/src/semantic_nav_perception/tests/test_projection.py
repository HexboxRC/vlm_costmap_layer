"""Known 3D targets projected to pixels independently of the inverse method."""
import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from semantic_nav_perception.projection import Camera, GroundProjector, UnavailableGeometry


class ProjectionTest(unittest.TestCase):
    def setUp(self):
        self.camera=Camera(640,480,380.,380.,320.,240.)
        self.rotation=np.array([[0,0,1],[-1,0,0],[0,-1,0.]])
        self.position=np.array([.15,0.,.15])

    def projector(self, **kwargs):
        settings=dict(pixel_error_bound=0.,max_sensitivity_m=.25,max_range_m=10.)
        settings.update(kwargs)
        return GroundProjector(self.camera,self.rotation,self.position,**settings)

    def image(self,point):
        x,y,z=self.rotation.T@(np.array(point)-self.position)
        return self.camera.fx*x/z+self.camera.cx,self.camera.fy*y/z+self.camera.cy

    def test_known_floor_points_and_vertical_glass_base(self):
        endpoints=[(3.,-1.,0.),(3.,1.,0.)]
        recovered=self.projector().visible_base([self.image(p) for p in endpoints],base_visible=True)
        for observed,expected in zip(recovered,endpoints):
            np.testing.assert_allclose([observed.x,observed.y],expected[:2],atol=1e-10)
        # Background lidar depth has no input into this reconstruction.
        self.assertNotAlmostEqual(recovered[0].x,6.)

    def test_translated_yawed_camera(self):
        angle=.7
        yaw=np.array([[math.cos(angle),-math.sin(angle),0.],
                      [math.sin(angle),math.cos(angle),0.],[0.,0.,1.]])
        self.rotation=yaw@self.rotation
        self.position=np.array([-2.,1.,.25])
        target=self.position+yaw@np.array([2.,.3,-.25])
        p=self.projector().point(*self.image(target))
        np.testing.assert_allclose([p.x,p.y],target[:2],atol=1e-10)

    def test_occluded_base_horizon_range_and_image_edge_abstain(self):
        proj=self.projector(pixel_error_bound=2.)
        with self.assertRaises(UnavailableGeometry):proj.visible_base([(300.,300.),(340.,300.)],base_visible=False)
        for pixel in [(320.,240.),(320.,239.),(0.,300.),(640.,300.),(float('nan'),300.)]:
            with self.subTest(pixel=pixel),self.assertRaises(UnavailableGeometry):proj.point(*pixel)
        with self.assertRaises(UnavailableGeometry):self.projector(max_range_m=1.).point(*self.image((3.,0.,0.)))

    def test_sensitivity_grows_with_range_and_is_not_hidden(self):
        proj=self.projector(pixel_error_bound=2.,max_sensitivity_m=2.)
        near=proj.point(*self.image((1.15,0.,0.)))
        far=proj.point(*self.image((4.15,0.,0.)))
        self.assertGreater(far.sensitivity_m,near.sensitivity_m*10)
        with self.assertRaises(UnavailableGeometry):self.projector(pixel_error_bound=2.).point(*self.image((4.15,0.,0.)))

    def test_valid_grass_polygon_preserves_geometry(self):
        targets=[(1.5,-.5,0.),(3.,-.5,0.),(3.,.5,0.),(1.5,.5,0.)]
        points=self.projector().ground_polygon([self.image(p) for p in targets])
        np.testing.assert_allclose([(p.x,p.y) for p in points],np.array(targets)[:,:2],atol=1e-10)

    def test_invalid_rotation_and_polygon_rejected(self):
        with self.assertRaises(ValueError):GroundProjector(self.camera,np.eye(3)*2,self.position,
            pixel_error_bound=1,max_sensitivity_m=.25,max_range_m=5)
        polygon=[self.image(p) for p in [(2.,-.5,0.),(3.,.5,0.),(2.,.5,0.),(3.,-.5,0.)]]
        with self.assertRaises(UnavailableGeometry):self.projector().ground_polygon(polygon)


if __name__=='__main__':unittest.main()
