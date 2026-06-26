#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class Mypubnode(Node):
    def __init__(self):
        self.count = 0
        super().__init__("talker")
        self.pub=self.create_publisher(String,"qmark_",10)
        self.get_logger().info("my node has been started")
        self.declare_parameter("timing",1.0)
        self.timer_peroid=self.get_parameter("timing").value
        self.timer=self,self.create_timer(self.timer_peroid,self.timer)

    def timer(self):
        
        msg=String()
        msg.data="Hallo world " + str(self.count)
        self.get_logger().info(msg.data)
        self.count += 1
        self.pub.publish(msg)
        ##self.get_logger().info("Publishing : i am publishing")

def main(args=None):
    rclpy.init(args=args)
    node=Mypubnode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()
