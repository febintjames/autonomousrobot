#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import sys

class CmdVelSerial(Node):

    def __init__(self):
        super().__init__('cmd_vel_serial_bridge')

        # Declare a parameter so you can change the port dynamically if needed
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        port_name = self.get_parameter('serial_port').value

        self.get_logger().info(f"Connecting to serial port: {port_name}...")
        
        try:
            self.serial_port = serial.Serial(
                port_name, 115200, timeout=1
            )
            self.get_logger().info(f"Successfully connected to {port_name}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {port_name}: {e}")
            self.get_logger().error("Ensure the USB is connected and you have permission (run: sudo usermod -aG dialout $USER)")
            sys.exit(1)

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        # Register a shutdown callback to close the serial port cleanly
        self.context.on_shutdown(self.on_shutdown)

    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Speed mapping
        linear_speed = int(abs(linear) * 255)
        linear_speed = max(0, min(255, linear_speed))
        
        angular_speed = int(abs(angular) * 255)
        angular_speed = max(0, min(255, angular_speed))
        
        # Determine movement command
        if linear > 0.05:
            self.get_logger().info(f"Forward command: {linear_speed}")
            cmd_str = f"F{linear_speed}\n"
        elif linear < -0.05:
            self.get_logger().info(f"Backward command: {linear_speed}")
            cmd_str = f"B{linear_speed}\n"
        elif angular > 0.2:
            self.get_logger().info(f"Left command: {angular_speed}")
            cmd_str = f"L{angular_speed}\n"
        elif angular < -0.2:
            self.get_logger().info(f"Right command: {angular_speed}")
            cmd_str = f"R{angular_speed}\n"
        else:
            self.get_logger().info("Stop command")
            cmd_str = "S\n"
        
        try:
            self.serial_port.write(cmd_str.encode())
        except Exception as e:
            self.get_logger().error(f"Failed to write to serial: {e}")

    def on_shutdown(self):
        self.get_logger().info("Shutting down serial bridge node...")
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
            self.get_logger().info("Serial port closed.")


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSerial()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
