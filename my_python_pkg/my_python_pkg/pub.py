#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class Mypubnode(Node):
    def __init__(self):
        super().__init__("publishing")
        self.publisher=self.create_publisher(String,"febin",10)
        self.get_logger().info("i am starting to publish")
        self.declare_parameter("time",0.5)
        self.param_value=self.get_parameter("time").value
        self.timer=self.create_timer(self.param_value,self.timer)

    def timer(self):
        msg=String()
        msg.data="hello my dear students"
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node=Mypubnode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
