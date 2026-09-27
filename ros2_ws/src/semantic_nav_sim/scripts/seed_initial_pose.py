#!/usr/bin/env python3
"""Seed AMCL only after activation; exit after it reports the requested pose."""
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState


def main():
    rclpy.init()
    args = rclpy.utilities.remove_ros_args(sys.argv)[1:]
    x, y, yaw = map(float, args)
    node = rclpy.create_node('seed_initial_pose')
    client = node.create_client(GetState, '/amcl/get_state')
    pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
    localized = []

    def receive(msg):
        position = msg.pose.pose.position
        if math.hypot(position.x-x, position.y-y) < 1.0:
            localized.append(True)

    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', receive, 10)
    future = None
    active = False
    last_publish = 0.
    deadline = time.monotonic() + 120.
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            if not active and future is None and client.service_is_ready():
                future = client.call_async(GetState.Request())
            rclpy.spin_once(node, timeout_sec=0.1)
            if future is not None and future.done():
                response = future.result()
                active = response.current_state.id == State.PRIMARY_STATE_ACTIVE
                future = None
            if localized:
                node.get_logger().info('Initial pose acknowledged by AMCL')
                return
            if (active and pub.get_subscription_count() > 0 and
                    node.get_clock().now().nanoseconds > 0 and
                    time.monotonic()-last_publish >= 1.):
                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = 'map'
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.pose.pose.position.x = x
                msg.pose.pose.position.y = y
                msg.pose.pose.orientation.z = math.sin(yaw/2)
                msg.pose.pose.orientation.w = math.cos(yaw/2)
                pub.publish(msg)
                last_publish = time.monotonic()
        raise RuntimeError('INFRASTRUCTURE: AMCL readiness/initial-pose timeout')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
