#!/usr/bin/env python3
"""Standalone stationary camera calibration captures; never run beside a pilot.

Gazebo truth is evaluation-only here. The deployment projector does not read it.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import numpy as np
import psutil
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'ros2_ws/src/semantic_nav_sim/scripts'))
from run_trials import assert_clean, teardown_stack


def assert_no_batch():
    for process in psutil.process_iter():
        try:
            if Path(process.cwd()).resolve() != root:
                continue
            if any(Path(arg).name in ('run_matched_pilot.py', 'run_campaign.py') for arg in process.cmdline()):
                raise RuntimeError('Matched batch is still alive; camera capture must wait')
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            continue


def rotation(q):
    x,y,z,w=q.x,q.y,q.z,q.w
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    args.out=args.out.resolve()
    if not args.out.is_relative_to(root):raise SystemExit('Capture artifacts must stay inside this repository')
    if args.out.exists():raise SystemExit('Refusing to overwrite capture evidence')
    assert_no_batch()
    assert_clean('projection capture startup')
    args.out.mkdir(parents=True)
    source=root/'ros2_ws/src/semantic_nav_sim/worlds/arena.sdf'
    tree=ET.parse(source);tree.find('./world/physics/real_time_factor').text='.5'
    world=(args.out/'arena.sdf').resolve();tree.write(world)
    for index,x in enumerate([-4.,-3.,-2.,-1.5]):
        assert_no_batch()
        directory=args.out/f'view_{index}';directory.mkdir()
        with (directory/'launch.log').open('w') as log:
            process=subprocess.Popen(['ros2','launch','semantic_nav_sim','robot_bringup.launch.py',
                f'x:={x}','y:=4.0','yaw:=0.0',f'world:={world}'],stdout=log,
                stderr=subprocess.STDOUT,start_new_session=True,cwd=root)
            node=listener=None
            try:
                rclpy.init()
                node=Node('projection_capture',parameter_overrides=[Parameter('use_sim_time',value=True)])
                buffer=Buffer();listener=TransformListener(buffer,node)
                latest={}
                for topic,kind,key in [('/camera',Image,'image'),('/camera_info',CameraInfo,'info'),
                                       ('/benchmark/robot_pose',PoseStamped,'truth'),('/scan',LaserScan,'scan')]:
                    node.create_subscription(kind,topic,lambda msg,key=key:latest.update({key:msg}),qos_profile_sensor_data)
                captured=False
                deadline=time.monotonic()+45
                while time.monotonic()<deadline:
                    rclpy.spin_once(node,timeout_sec=.1)
                    if len(latest)!=4:continue
                    image,info,truth=latest['image'],latest['info'],latest['truth']
                    if image.header.stamp.sec<2:continue
                    if image.header.frame_id!=info.header.frame_id:
                        raise RuntimeError('Image and calibration frame mismatch')
                    if (image.width,image.height)!=(info.width,info.height):
                        raise RuntimeError('Image and calibration dimensions mismatch')
                    if any(abs(d)>1e-9 for d in info.d):
                        raise RuntimeError('Unrectified distortion unsupported')
                    try:transform=buffer.lookup_transform('base_footprint',image.header.frame_id,rclpy.time.Time()).transform
                    except Exception:continue
                    if image.encoding not in ('rgb8','bgr8'):
                        raise RuntimeError(f'Unsupported image encoding {image.encoding}')
                    rgb=PILImage.frombytes('RGB',(image.width,image.height),bytes(image.data),
                        'raw','RGB' if image.encoding=='rgb8' else 'BGR',image.step)
                    rgb.save(directory/'camera.png')
                    rb=rotation(truth.pose.orientation);rt=rotation(transform.rotation)
                    pt=np.array([transform.translation.x,transform.translation.y,transform.translation.z])
                    pb=np.array([truth.pose.position.x,truth.pose.position.y,truth.pose.position.z])
                    metadata=dict(width=image.width,height=image.height,k=list(info.k),d=list(info.d),
                        image_frame=image.header.frame_id,truth_frame=truth.header.frame_id,
                        image_sim_s=image.header.stamp.sec+image.header.stamp.nanosec*1e-9,
                        truth_sim_s=truth.header.stamp.sec+truth.header.stamp.nanosec*1e-9,
                        rotation_world_from_optical=(rb@rt).tolist(),camera_position=(pb+rb@pt).tolist(),
                        spawn_x=x,world_sha256=hashlib.sha256(world.read_bytes()).hexdigest(),
                        evaluation_only=True)
                    (directory/'calibration.json').write_text(json.dumps(metadata,indent=2)+'\n')
                    scan=latest['scan']
                    (directory/'scan.json').write_text(json.dumps(dict(
                        frame=scan.header.frame_id,
                        sim_s=scan.header.stamp.sec+scan.header.stamp.nanosec*1e-9,
                        angle_min=scan.angle_min,angle_increment=scan.angle_increment,
                        range_min=scan.range_min,range_max=scan.range_max,
                        ranges=[float(r) if np.isfinite(r) else None for r in scan.ranges]),indent=2)+'\n')
                    print(f'Captured {directory}',flush=True);captured=True;break
                if not captured:raise RuntimeError('Bounded camera-capture timeout')
            finally:
                try:
                    if listener is not None:listener.unregister()
                    if node is not None:node.destroy_node()
                    rclpy.try_shutdown()
                finally:
                    teardown_stack(process,log)
        assert_clean('projection capture teardown')


if __name__=='__main__':main()
