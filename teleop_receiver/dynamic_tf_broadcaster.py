import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Twist
import tf2_ros
import math
import numpy as np

def compute_distance_from_odom(wheel1, wheel2, wheel3, wheel4):
    x = 2 * np.pi / 1140 * (wheel1 + wheel2 + wheel3 + wheel4) * 0.08 / 4
    y = 2 * np.pi / 1140 * (-wheel1 + wheel2 - wheel3 + wheel4) * 0.08 / 4
    omega = 2 * np.pi / 1140 * (-wheel1 + wheel2 + wheel3 - wheel4) * 0.08 / 4
    return x, y, omega

def compute_transformations(old_position, wheel1, wheel2, wheel3, wheel4):
    x, y, omega = compute_distance_from_odom(wheel1, wheel2, wheel3, wheel4)
    tx = x - old_position[0]
    ty = y - old_position[1]
    tomega = omega - old_position[2]
    return tx, ty, tomega

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
        self.wheel1_speed = 0.0
        self.wheel2_speed = 0.0
        self.wheel3_speed = 0.0
        self.wheel4_speed = 0.0

        # Create a subscriber to receive wheel speed data
        self.subscription = self.create_subscription(
            Twist,
            '/receiver_node_topic',
            self.receiver_callback,
            10
        )

        # Set up a timer to call update every 0.1 seconds
        self.timer = self.create_timer(0.1, self.update)

    def setup_serial(self):
        try:
            self.serial_connection = serial.Serial(self.get_parameter('serial_port').value,
                                                   self.get_parameter('baud_rate').value, timeout=1)
            self.get_logger().info('Serial Connection Established')
        except Exception as e:
            self.get_logger().error('Failed to establish connection: ' + str(e))

    def update(self):
        # Update transform
        self.broadcast_dynamic_transform()

        try:
            if self.serial_connection.in_waiting > 0:
                serial_data = self.serial_connection.readline().decode('utf-8').strip()
                self.get_logger().info(f"Received serial data: {serial_data}")

                if serial_data.startswith("OUT: "):
                    wheel_speeds = serial_data[4:].split(',')
                    self.wheel1_speed, self.wheel2_speed, self.wheel3_speed, self.wheel4_speed = map(float, wheel_speeds)

        except Exception as e:
            self.get_logger().error(f"Failed to read from serial connection: {str(e)}")


    def broadcast_dynamic_transform(self):
        # Create a TransformStamped message for dynamic transform
        t = TransformStamped()

        # Set the timestamp and frame names
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'  # Parent frame
        t.child_frame_id = 'base_footprint'  # Child frame

        # Compute the new position and orientation
        delta_x, delta_y, delta_theta = compute_transformations(
            (self.x, self.y, self.theta),
            self.wheel1_speed,
            self.wheel2_speed,
            self.wheel3_speed,
            self.wheel4_speed
        )

        # Update the current position and orientation
        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        # Set translation and rotation
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)

        # Send the dynamic transform
        self.broadcaster.sendTransform(t)

    def receiver_callback(self, msg: Twist):
        # Update wheel speeds based on the message from ReceiverNode
        self.wheel1_speed = msg.linear.x
        self.wheel2_speed = msg.linear.y
        self.wheel3_speed = msg.angular.z
        self.wheel4_speed = msg.angular.z

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