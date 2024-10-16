#!/usr/bin/env python3

import rclpy
from hgext.histedit import message
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class ReceiverNode(Node):
    def __init__(self):
        super().__init__('receiver_node')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        try:
            self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info('Serial Connection Established')
        except Exception as e:
            self.get_logger().info('Failed to establish connection: ' + str(e))
            return
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.receiver_callback,
            10)

    def receiver_callback(self, msg: Twist):
        # Log the linear and angular velocities
        # self.get_logger().info(f"Received /cmd_vel: Linear: x={msg.linear.x}, y={msg.linear.y}, z={msg.linear.z} | "
        #                        f"Angular: x={msg.angular.x}, y={msg.angular.y}, z={msg.angular.z}")

        # Calculate and log wheel velocities
        wheel_front_left, wheel_front_right, wheel_back_left, wheel_back_right = self.compute_wheel_velocities(msg)

        # Send the wheel velocities to the Arduino
        try:
            message_str = f"IN: {round(wheel_front_left, 1)},{round(wheel_front_right, 1)},{round(wheel_back_left, 1)},{round(wheel_back_right, 1)} \n"
            self.get_logger().info(message_str)

            self.serial_connection.write(message_str.encode())
        except Exception as e:
            self.get_logger().info('Failed to send message: ' + str(e))

    def compute_wheel_velocities(self, msg: Twist):
        # Calculate the angular velocity of the wheels in rad/s
        wheel_front_left = (msg.linear.x - msg.linear.y - msg.angular.z * 0.23) / 0.04
        wheel_front_right = (msg.linear.x + msg.linear.y + msg.angular.z * 0.23) / 0.04
        wheel_back_left = (msg.linear.x + msg.linear.y - msg.angular.z * 0.23) / 0.04
        wheel_back_right = (msg.linear.x - msg.linear.y + msg.angular.z * 0.23) / 0.04
        return wheel_front_left, wheel_front_right, wheel_back_left, wheel_back_right

def main(args=None):
    rclpy.init(args=args)
    cmd_vel_listener = ReceiverNode()

    try:
        rclpy.spin(cmd_vel_listener)
    except KeyboardInterrupt:
        pass

    cmd_vel_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
