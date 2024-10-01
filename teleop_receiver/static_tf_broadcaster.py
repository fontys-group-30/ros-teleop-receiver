#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class StaticTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)


        # Static transform from 'odom' to 'base_footprint'
        self.broadcast_static_transform(('odom', 'base_footprint', 0, 0, 0))
        # Static transform from 'base_footprint' to 'laser'
        self.broadcast_static_transform(('base_footprint', 'laser', 0.1, 0.0, 0.15))

    def broadcast_static_transform(self, transformation):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = transformation[0]
        t.child_frame_id = transformation[1]
        t.transform.translation.x = float(transformation[2])
        t.transform.translation.y = float(transformation[3])
        t.transform.translation.z = float(transformation[4])
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(t)
        self.get_logger().info(f'Broadcasting static transform from {transformation[0]} to {transformation[1]}')


def main():
    # Initialize the ROS client library
    rclpy.init()

    # Create the static transform broadcaster node
    node = StaticTransformBroadcaster(('world', 'odom', 0, 0, 0))

    # Keep the node alive
    rclpy.spin(node)

    # Shutdown and clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
