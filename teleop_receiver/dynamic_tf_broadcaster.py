import math
import rclpy
import serial
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node


def compute_distance_from_odom(wheel_front_left, wheel_front_right, wheel_back_left, wheel_back_right):
    r = 0.04  # Wheel radius in meters
    L = 0.08  # Distance from center to front/back wheels
    W = 0.15  # Distance from center to side wheels

    # Compute velocities in the robot's local frame
    x = (((wheel_front_left + wheel_front_right + wheel_back_left + wheel_back_right)/4)/1440) * (r * 2 * np.pi)
    y = (((-wheel_front_left + wheel_front_right + wheel_back_left - wheel_back_right)/4)/1440) * (r * 2 * np.pi)

    # Compute the angular velocity, accounting for both length (L) and width (W)
    theta = 2.98*(r / (4 * (L + W))) * (-wheel_front_left + wheel_front_right - wheel_back_left + wheel_back_right) / 1440

    return x, y, theta


def compute_transformations(old_position, wheel_front_left, wheel_front_right, wheel_back_left, wheel_back_right):
    x, y, theta = compute_distance_from_odom(wheel_front_left, wheel_front_right, wheel_back_left, wheel_back_right)
    delta_x = x - old_position[0]
    delta_y = y - old_position[1]
    delta_theta = theta - old_position[2]

    return delta_x, delta_y, delta_theta


class DynamicTransformBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_tf_broadcaster')
        self.serial_connection = None
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        # Set up serial connection
        self.setup_serial()

        # Initialize transform broadcaster
        self.broadcaster = tf2_ros.TransformBroadcaster(self)

        # Initialize position and orientation
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Initialize wheel speeds
        self.wheel_front_left = 0.0
        self.wheel_front_right = 0.0
        self.wheel_back_left = 0.0
        self.wheel_back_right = 0.0

        # Set up a timer to call update every 0.1 seconds
        self.timer = self.create_timer(0.1, self.update)

    def setup_serial(self):
        try:
            self.serial_connection = serial.Serial(self.get_parameter('serial_port').value, self.get_parameter('baud_rate').value, timeout=1)
            self.get_logger().info('Serial Connection Established')
        except Exception as e:
            self.get_logger().error('Failed to establish connection: ' + str(e))

    def update_position(self):
        # Compute the new position and orientation
        delta_x, delta_y, delta_theta = compute_transformations(
            (self.x, self.y, self.theta),
            self.wheel_front_left,
            self.wheel_front_right,
            self.wheel_back_left,
            self.wheel_back_right
        )

        # Update the current position and orientation
        self.x += delta_x * math.cos(self.theta) - delta_y * math.sin(self.theta)
        self.y += delta_x * math.sin(self.theta) + delta_y * math.cos(self.theta)
        self.theta += delta_theta

        self.get_logger().info(f"Theta: {self.theta}, Encoder Left Front {self.wheel_front_left}, Encoder Right Front {self.wheel_front_right}, Encoder Left Back {self.wheel_back_left}, Encoder Right Back {self.wheel_back_right}")

    def update(self):
        # Update position
        self.update_position()
        
        # Dynamic transform from 'odom' to 'base_footprint'
        self.broadcast_dynamic_transform('odom', 'base_footprint', self.x, self.y, self.theta)

        # Dynamic transform from 'base_footprint' to 'base_link'
        self.broadcast_dynamic_transform('base_footprint', 'base_link', 0.0, 0.0, 0.0)
        
        # Dynamic transform from 'base_link' to 'laser'
        self.broadcast_dynamic_transform('base_link', 'laser', 0.0, 0.0, math.pi / 4)

        # Dynamic transform for each wheel
        # self.broadcast_dynamic_transform('base_link', 'left_front_wheel', 0.0475, 0.13, self.wheel_front_left)
        # self.broadcast_dynamic_transform('base_link', 'right_front_wheel', 0.0475, -0.13, self.wheel_front_right)
        # self.broadcast_dynamic_transform('base_link', 'left_back_wheel', -0.0475, 0.13, self.wheel_back_left)
        # self.broadcast_dynamic_transform('base_link', 'right_back_wheel', -0.0475, -0.13, self.wheel_back_right)

        try:
            if self.serial_connection.in_waiting > 0:
                serial_data = self.serial_connection.readline().decode('utf-8').strip()

                if serial_data.startswith("OUT: "):
                    wheel_speeds = serial_data[4:].split(',')
                    self.wheel_front_left, self.wheel_front_right, self.wheel_back_left, self.wheel_back_right = map(float, wheel_speeds)

        except Exception as e:
            self.get_logger().error(f"Failed to read from serial connection: {str(e)}")

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

        # Set rotation using quaternion
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
