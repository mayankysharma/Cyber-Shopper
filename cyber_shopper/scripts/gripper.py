#! /usr/bin/env python3

import rclpy
import time
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from sympy import symbols,cos,sin,simplify,Matrix,pprint,evalf,diff,sqrt
from functools import partial
from mpl_toolkits.mplot3d import axes3d, Axes3D
from matplotlib import pyplot as plt
import sys

from std_srvs.srv import SetBool
class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SetBool, '/vacuum_gripper/switch')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()

    def send_request(self, status):
        self.req.data = status
        self.future = self.cli.call_async(self.req)
        #rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    status = True
    response = minimal_client.send_request(status)
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()