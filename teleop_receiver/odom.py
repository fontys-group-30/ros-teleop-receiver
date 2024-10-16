import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion
from rclpy.qos import QoSProfile
import math
import serial
import time

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odometry_publisher')

        # Create a publisher for the Odometry message
        self.wheel_back_right = None
        self.wheel_back_left = None
        self.wheel_front_right = None
        self.wheel_front_left = None
        qos_profile = QoSProfile(depth=10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos_profile)

        # Timer to publish the odometry
        self.timer = self.create_timer(0.1, self.publish_odometry)

        # Initialize pose variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Wheel parameters
        self.wheel_radius = 0.1  # radius of the wheel in meters
        self.wheel_base = 0.5     # distance between the wheels in meters

        # Previous encoder readings
        self.prev_wheel_front_left = 0
        self.prev_wheel_front_right = 0
        self.prev_wheel_back_left = 0
        self.prev_wheel_back_right = 0

        # Setup the serial connection
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2)  # Wait for the serial connection to initialize

    def read_encoders(self):
        try:
            if self.serial_port.in_waiting > 0:
                serial_data = self.serial_port.readline().decode('utf-8').strip()

                if serial_data.startswith("OUT: "):
                    wheel_speeds = serial_data[4:].split(',')
                    self.wheel_front_left, self.wheel_front_right, self.wheel_back_left, self.wheel_back_right = map(float, wheel_speeds)
                    return self.wheel_front_left, self.wheel_front_right, self.wheel_back_left, self.wheel_back_right

        except Exception as e:
            self.get_logger().error(f"Failed to read from serial connection: {str(e)}")
        return None, None, None, None

    def update_pose(self, wheel_front_left, wheel_front_right, wheel_back_left, wheel_back_right):
        # Calculate the distance traveled by each wheel
        left_front_distance = (wheel_front_left - self.prev_wheel_front_left) * (2 * math.pi * self.wheel_radius / 1440.0)  # 1440 pulses per revolution
        right_front_distance = (wheel_front_right - self.prev_wheel_front_right) * (2 * math.pi * self.wheel_radius / 1440.0)
        left_back_distance = (wheel_back_left - self.prev_wheel_back_left) * (2 * math.pi * self.wheel_radius / 1440.0)
        right_back_distance = (wheel_back_right - self.prev_wheel_back_right) * (2 * math.pi * self.wheel_radius / 1440.0)

        # Update previous encoder values
        self.prev_wheel_front_left = wheel_front_left
        self.prev_wheel_front_right = wheel_front_right
        self.prev_wheel_back_left = wheel_back_left
        self.prev_wheel_back_right = wheel_back_right

        # Calculate the robot's movement in local coordinates
        delta_local_x = (left_front_distance + right_front_distance + left_back_distance + right_back_distance) / 4.0
        delta_local_y = (-left_front_distance + right_front_distance + left_back_distance - right_back_distance) / 4.0
        delta_theta = (-left_front_distance + right_front_distance - left_back_distance + right_back_distance) / (4.0 * self.wheel_base)

        # Update the robot's pose
        self.theta = (self.theta + delta_theta) % (2 * math.pi)
        delta_global_x = delta_local_x * math.cos(self.theta) - delta_local_y * math.sin(self.theta)
        delta_global_y = delta_local_x * math.sin(self.theta) + delta_local_y * math.cos(self.theta)
        self.x += delta_global_x
        self.y += delta_global_y

    def publish_odometry(self):
        # Read encoder values
        wheel_front_left, wheel_front_right, wheel_back_left, wheel_back_right = self.read_encoders()

        if wheel_front_left is not None and wheel_front_right is not None and wheel_back_left is not None and wheel_back_right is not None:
            self.update_pose(wheel_front_left, wheel_front_right, wheel_back_left, wheel_back_right)

        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Set the position and orientation
        odom_msg.pose.pose = Pose()
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = self.get_orientation(self.theta)

        # Publish the Odometry message
        self.odom_pub.publish(odom_msg)

    def get_orientation(self, theta):
        # Convert theta to quaternion
        q = Quaternion()
        q.w = math.cos(theta / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(theta / 2.0)
        return q

def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdometryPublisher()

    try:
        rclpy.spin(odom_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        odom_publisher.serial_port.close()  # Close the serial port
        odom_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()