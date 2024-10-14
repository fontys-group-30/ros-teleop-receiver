#!/usr/bin/env python3
import math
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class StaticTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Static transform from 'map' to 'odom'
        self.broadcast_static_transform('map', 'odom', 0.0, 0.0, 0.0, 0.0)

        # Static transform from 'odom' to 'base_footprint'
        self.broadcast_static_transform('odom', 'base_footprint', 0.0, 0.0, 0.0, 0.0)

        # Static transform from 'base_footprint' to 'base_link'
        self.broadcast_static_transform('base_footprint', 'base_link', 0.0, 0.0, 0.0, 0.0)

        # Static transform from 'base_link' to 'laser'
        self.broadcast_static_transform('base_link', 'laser', 0.1, 0.0, 0.15, math.pi / 2)

    def broadcast_static_transform(self, parent_frame, child_frame, x, y, z, theta):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(theta / 2)
        t.transform.rotation.w = math.cos(theta / 2)
        self.static_broadcaster.sendTransform(t)
        self.get_logger().info(f'Broadcasting static transform from {parent_frame} to {child_frame}')


def main():
    rclpy.init()
    node = StaticTransformBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
