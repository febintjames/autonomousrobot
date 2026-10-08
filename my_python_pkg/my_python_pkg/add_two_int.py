#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class Myserver(Node):
    def __init__(self):
        super().__init__("febinserver")
        self.server=self.create_service(AddTwoInts,"febin",self.callback)

    def callback(self,request,response):
        response.sum =request.a +request.b
        self.get_logger().info(str(request.a)+"+"+ str(request.b)+ "=" + str(response.sum))
        return response


def main(args=None):
    rclpy.init(args=args)
    node=Myserver()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()