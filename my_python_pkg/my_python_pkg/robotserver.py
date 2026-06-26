#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class Myserver(Node):
    def __init__(self):
        super().__init__("febinserver")
        self.server=self.create_service(SetBool,"febin",self.callback)

    def callback(self,request,response):
        if request.data ==True:
            response.success =True
            response.message="robot powered on "
        else:
            response.success=True
            response.message ="robot powered off "
        return response


def main(args=None):
    rclpy.init(args=args)
    node=Myserver()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__=="__main__":
    main()