#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import serial
from threading import Lock

class SerialController(Node):
    def __init__(self):
        super().__init__('serial_controller')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10)
        self.steering_publisher = self.create_publisher(Float64, '/servo_steering_controller/command', 10)
        self.wheel_publisher = self.create_publisher(Float64, '/rear_wheel_controller/command', 10)
        self.wheelbase_length = 0.26
        self.wheelbase_width = 0.213

        self.declare_parameter('serial_port', value="/dev/ttyAMA10")
        self.serial_port_name = self.get_parameter('serial_port').value

        self.declare_parameter('baud_rate', value=460800)
        self.baud_rate = self.get_parameter('baud_rate').value

        self.declare_parameter('serial_debug', value=False)
        self.debug_serial_cmds = self.get_parameter('serial_debug').value
        if self.debug_serial_cmds:
            self.get_logger().info("Serial debug enabled")

        self.get_logger().info(f"Connecting to port {self.serial_port_name} at {self.baud_rate}.")
        self.conn = serial.Serial(self.serial_port_name, self.baud_rate, timeout=0.2)
        self.get_logger().info(f"Connected to {self.conn.port}")
        self.mutex = Lock()

        self.last_steering_angle = 0.0
        self.last_wheel_velocity = 0.0

    def joint_states_callback(self, msg):
        try:
            steering_joint_name = "front_left_wheel_steer_joint"
            wheel_joint_name = "back_left_wheel_joint"

            steering_index = msg.name.index(steering_joint_name)
            wheel_index = msg.name.index(wheel_joint_name)

            current_steering_angle = msg.position[steering_index]
            current_wheel_velocity = msg.velocity[wheel_index]

            steering_msg = Float64()
            steering_msg.data = current_steering_angle
            self.steering_publisher.publish(steering_msg)

            wheel_msg = Float64()
            wheel_msg.data = current_wheel_velocity
            self.wheel_publisher.publish(wheel_msg)

            if current_steering_angle != self.last_steering_angle or current_wheel_velocity != self.last_wheel_velocity:
                self.send_serial_data(current_steering_angle, current_wheel_velocity)
                self.last_steering_angle = current_steering_angle
                self.last_wheel_velocity = current_wheel_velocity

        except ValueError as e:
            self.get_logger().error(f"Joint name not found: {e}")

    def send_command(self, cmd_string):
        self.mutex.acquire()
        try:
            cmd_string += "\r"
            self.conn.write(cmd_string.encode("utf-8"))
            if self.debug_serial_cmds:
                self.get_logger().info(f"Sent: {cmd_string}")

            c = ''
            value = ''
            while c != '\r':
                c = self.conn.read(1).decode("utf-8")
                if c == '':
                    self.get_logger().error(f"Error: Serial timeout on command: {cmd_string}")
                    return ''
                value += c

            value = value.strip('\r')
            if self.debug_serial_cmds:
                self.get_logger().info(f"Received: {value}")
            return value
        finally:
            self.mutex.release()

    def close_conn(self):
        self.conn.close()

    def send_serial_data(self, steering_angle, wheel_velocity):
        data = f"{steering_angle:.3f};{wheel_velocity:.3f}\n"
        self.send_command(data)
        self.get_logger().info(f"Sent data to serial: {data.strip()}")

def main(args=None):
    rclpy.init(args=args)
    serial_controller = SerialController()
    rclpy.spin(serial_controller)
    serial_controller.destroy_node()
    serial_controller.close_conn()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
