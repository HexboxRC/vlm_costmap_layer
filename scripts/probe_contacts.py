#!/usr/bin/env python3
"""Bounded free-ground / deliberate-glass-contact integration diagnostic."""
import csv
import json
from pathlib import Path
import sys
import time
import subprocess
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'ros2_ws/src/semantic_nav_sim/scripts'))
import run_trials
from contact_metrics import classify_pair
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import Contacts
root=Path(__file__).resolve().parents[1]
out=root/'results/phase0'/('contact_probe_'+str(time.monotonic_ns()))
out.mkdir(exist_ok=True)
run_trials.assert_clean('contact probe startup')
log=(out/'launch.log').open('w')
p=subprocess.Popen(['ros2','launch','semantic_nav_sim','robot_bringup.launch.py','x:=-1.0','y:=4.0','yaw:=0.0'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init()
node=Node('contact_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
pub=node.create_publisher(Twist,'/cmd_vel',10)
counts={'ground':0,'obstacle':0,'irrelevant':0}; samples=[]
def callback(msg):
    for c in msg.contacts:
        kind=classify_pair(c.collision1.name,c.collision2.name)
        counts[kind]+=1
        if len(samples)<5 or kind!='ground':
            samples.append({'sim_s':node.get_clock().now().nanoseconds*1e-9,'kind':kind,'collision1':c.collision1.name,'collision2':c.collision2.name,'depths':list(c.depths)})
node.create_subscription(Contacts,'/benchmark/contacts',callback,1000)
node.create_subscription(Contacts,'/benchmark/ground_contacts',callback,qos_profile_sensor_data)
passed=False
try:
    deadline=time.monotonic()+90
    free_end=None
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
        now=node.get_clock().now().nanoseconds*1e-9
        if counts['ground'] and free_end is None:
            free_end=now+2
            print('Ground contact stream received at sim',now,flush=True)
        if free_end is not None and now>=free_end:
            if counts['obstacle']:
                passed=True;break
            cmd=Twist();cmd.linear.x=.25;pub.publish(cmd)
        elif counts['obstacle']:
            raise RuntimeError('False collision during stationary free-space check')
    pub.publish(Twist())
finally:
    (out/'contacts.json').write_text(json.dumps({'passed':passed,'counts':counts,'samples':samples},indent=2))
    node.destroy_node();rclpy.try_shutdown();run_trials.teardown_stack(p,log);log.close()
print(json.dumps({'passed':passed,'counts':counts}))
if not passed:raise SystemExit('Contact sensor diagnostic failed')
