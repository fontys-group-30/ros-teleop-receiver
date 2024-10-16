import math
import rclpy
import serial
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node


def compute_velocity(wheel_front_left, wheel_front_right, wheel_back_left, wheel_back_right):
    r = 0.04  # Wheel radius in meters
    L = 0.08  # Distance from center to front/back wheels
    W = 0.15  # Distance from center to side wheels

    # Compute velocities in the robot's local frame
    vx = (((wheel_front_left + wheel_front_right + wheel_back_left + wheel_back_right)/4)/1440) * (r * 2 * np.pi)
    vy = (((-wheel_front_left + wheel_front_right + wheel_back_left - wheel_back_right)/4)/1440) * (r * 2 * np.pi)

    # Compute the angular velocity, accounting for both length (L) and width (W)
    vtheta = 2 * math.pi * (r / (4 * (L + W))) * (-wheel_front_left + wheel_front_right - wheel_back_left + wheel_back_right) / 1440

    return vx, vy, vtheta


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
        delta_t = 0.1

        # Compute the new position and orientation
        vel_x, vel_y, vel_theta = compute_velocity(
            self.wheel_front_left,
            self.wheel_front_right,
            self.wheel_back_left,
            self.wheel_back_right
        )

        # Update the current position and orientation
        # self.theta += vel_theta * delta_t
        # self.x += vel_x * delta_t
        # self.y += vel_y * delta_t


        self.theta = np.mod(self.theta + vel_theta * delta_t, 2 * np.pi)
        delta_x = (vel_x * math.cos(self.theta) - vel_y * math.sin(self.theta)) * delta_t
        delta_y = (vel_x * math.sin(self.theta) + vel_y * math.cos(self.theta)) * delta_t
        
        self.x += delta_x
        self.y += delta_y
        # if 0 <= self.theta < np.pi/4:
            # self.x += delta_x
            # self.y += delta_y
        # elif np.pi/4 <= self.theta < np.pi/2:
        #     self.x += -delta_y
        #     self.y += delta_x
        # elif np.pi/2 <= self.theta < np.pi*3/4:
        #     self.x += -delta_x
        #     self.y += -delta_y
        # else:
        #     self.x += delta_y
        #     self.y += -delta_x
        #

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

        try:
            if self.serial_connection.in_waiting > 0:
                serial_data = self.serial_connection.readline().decode('utf-8').strip()

                if serial_data.startswith("OUT: "):
                    self.old_wheel_front_left = self.wheel_front_left
                    self.old_wheel_front_right = self.wheel_front_right
                    self.old_wheel_back_left = self.wheel_back_left
                    self.old_wheel_back_right = self.wheel_back_right

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
