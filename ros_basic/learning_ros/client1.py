#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rosbasic_msgs.srv import BaiTap1
import sys 

from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter


class SimpleServiceClient(Node):
    def __init__(self):
        super().__init__("simple_service_client")
        self.client_ = self.create_client(BaiTap1, "Car")

        self.declare_parameter("simple_int_param", 28)
        
        

        while not self.client_.wait_for_service(timeout_sec=1.0):  # wait for service function, 

            self.get_logger().info("Service not available, waiting again...")
        # out loop if wait_for_service = true
        
        self.req_ = BaiTap1.Request() # message type
        self.req_.distance_x = 7.0
        self.req_.distance_y = 7.0
        self.req_.distance_lidar = 5.0
        self.req_.speed=1.0
        self.req_.ok=False

        # self.future_ = self.client_.call_async(self.req_) # send the request meg to service server and immediately returns a result 
        # # future variable implements a mechanism to store the output of asynchronous functions like this one
        self.future = self.client.call_async(self.req_)
        print("Service responded:", self.future.msg)
        self.future_.add_done_callback(self.responseCallback)

        # self.add_on_set_parameters_callback(self.paramChangeCallback)
   
    def responseCallback(self, future):
        self.get_logger().info("Service Response %s" % future.result().msg)

    # def paramChangeCallback(self, params):
    #     result = SetParametersResult()

    #     for param in params:

    #         if param.name == "simple_int_param" and param.type_ == Parameter.Type.INTEGER:
    #             self.get_logger().info("Param simple_int_param changed! New value is %d" % param.value)
    #             result.successful = True


    #     return result

def main():
    rclpy.init()
    # use sys lib to access to the parameter of the main. 
    # if the script has been started correctly, so with the correct number of arguments 
    simple_service_client = SimpleServiceClient() # argv[1] only name of the script
    rclpy.spin(simple_service_client)
    simple_service_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()