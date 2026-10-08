#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts 

class MyServiceClient(Node):
    def __init__(self):
        super().__init__("client_node")
        self.client=self.create_client(AddTwoInts,"febin")
        
    def call_service(self):
        while not self.client.wait_for_service(timeout_sec=1.0):

            self.get_logger().warn("Waiting for service...")

        request = AddTwoInts.Request()
        request.a = 8
        request.b = 2

        future = self.client.call_async(request)
        future.add_done_callback(self.response_)
    def response_(self,future):
        reponse =future.result()
        self.get_logger().info(str(reponse.sum))

def main(args=None):
    rclpy.init(args=args)
    node = MyServiceClient()
    node.call_service()   # SEND REQUEST
    rclpy.spin(node)
    rclpy.shutdown()

if __name__=="__main__":
    main()