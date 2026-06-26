#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class Mysubnode(Node):
    def __init__(self):
        super().__init__("febintjames")
        self.create_subscription(String,"qmark_",self.timer,100)
        

    def timer(self,msg):
        self.get_logger().info(msg.data)




def main(args=None):
    rclpy.init(args=args)
    node=Mysubnode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()