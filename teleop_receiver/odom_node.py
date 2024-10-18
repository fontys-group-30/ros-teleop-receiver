import math
import rclpy
import serial
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def compute_velocity(wheel_front_left, wheel_front_right, wheel_back_left, wheel_back_right):
    r = 0.04  # Wheel radius in meters
    L = 0.08  # Distance from center to front/back wheels
    W = 0.15  # Distance from center to side wheels

    # Compute velocities in the robot's local frame
    vx = (((wheel_front_left + wheel_front_right + wheel_back_left + wheel_back_right)/4)/1440) * (r * 2 * np.pi)
    vy = (((-wheel_front_left + wheel_front_right + wheel_back_left - wheel_back_right)/4)/1440) * (r * 2 * np.pi)

    # Compute the angular velocity, accounting for both length (L) and width (W)
    vtheta = math.pi * (r / (4 * (L + W))) * (-wheel_front_left + wheel_front_right - wheel_back_left + wheel_back_right) / 1440

    return vx, vy, vtheta

class DynamicTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_node')
        self.serial_connection = None
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        # Set up serial connection
        self.setup_serial()

        # Initialize transform broadcaster
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize odometry publisher
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)

        # Initialize position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.wheel_front_left = 0.0
        self.wheel_front_right = 0.0
        self.wheel_back_left = 0.0
        self.wheel_back_right = 0.0

        # Set up a timer to call update every 0.1 seconds
        self.timer = self.create_timer(0.1, self.update)

    def setup_serial(self):
        self.serial_connection = serial.Serial(self.get_parameter('serial_port').value,
                                               self.get_parameter('baud_rate').value, timeout=1)

    def update_position(self):
        delta_t = 0.1

        # Compute the new position and orientation
        vel_x, vel_y, vel_theta = compute_velocity(
            self.wheel_front_left,
            self.wheel_front_right,
            self.wheel_back_left,
            self.wheel_back_right
        )

        self.theta = np.mod(self.theta + vel_theta * delta_t, 2 * np.pi)

        modified_theta = self.theta * 2

        delta_x = (vel_x * math.cos(modified_theta) - vel_y * math.sin(modified_theta)) * delta_t
        delta_y = (vel_x * math.sin(modified_theta) + vel_y * math.cos(modified_theta)) * delta_t

        self.x += delta_x
        self.y += delta_y

    def update(self):
        # Update position
        self.update_position()

        # Broadcast dynamic transforms
        self.broadcast_dynamic_transform('odom', 'base_footprint', self.x, self.y, self.theta)
        self.broadcast_dynamic_transform('base_footprint', 'base_link', 0.0, 0.0, 0.0)
        self.broadcast_dynamic_transform('base_link', 'laser', 0.0, 0.0, math.pi / 4)

        # Publish odometry information
        self.publish_odometry()

        try:
            if self.serial_connection.in_waiting > 0:
                serial_data = self.serial_connection.readline().decode('utf-8').strip()

                if serial_data.startswith("OUT: "):
                    wheel_speeds = serial_data[4:].split(',')
                    self.wheel_front_left, self.wheel_front_right, self.wheel_back_left, self.wheel_back_right = map(
                        float, wheel_speeds)

        except Exception as e:
            self.get_logger().error(f"Failed to read from serial connection: {str(e)}")

    def publish_odometry(self):
        # Create an Odometry message
        odom_msg = Odometry()

        # Set the timestamp and frame IDs
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'  # Parent frame
        odom_msg.child_frame_id = 'base_footprint'  # Child frame

        # Set position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Set rotation using quaternion_from_euler
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta)

        # Set linear velocity
        odom_msg.twist.twist.linear.x = (
                                                    self.wheel_front_left + self.wheel_front_right + self.wheel_back_left + self.wheel_back_right) / 4  # Average wheel speed for linear x
        odom_msg.twist.twist.linear.y = 0.0  # Assuming no lateral velocity
        odom_msg.twist.twist.angular.z = (
                                                     self.wheel_front_right - self.wheel_front_left + self.wheel_back_right - self.wheel_back_left) / 4  # Simplified angular velocity

        # Publish the Odometry message
        self.odom_publisher.publish(odom_msg)

    def broadcast_dynamic_transform(self, parent_frame, child_frame, x, y, theta):
        # Create a TransformStamped message for dynamic transform
        t = TransformStamped()

        # Set the timestamp and frame names
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame  # Parent frame
        t.child_frame_id = child_frame  # Child frame

        # Set translation
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = 0.0

        # Set rotation using quaternion_from_euler
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(theta)
        t.transform.rotation.w = math.cos(theta)

        # Send the dynamic transform
        self.broadcaster.sendTransform(t)


def main():
    # Initialize the ROS2 Python client library
    rclpy.init()

    # Create the node
    node = DynamicTransformBroadcaster()

    # Keep the node running to broadcast transforms
    rclpy.spin(node)

    # Shutdown and cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()