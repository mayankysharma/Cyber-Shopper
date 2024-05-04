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
class Circle_UR5(Node):

    def __init__(self):
        super().__init__('node_circle')
        # self.q_pub1=[-3.14,-3.14,-1.74,-3.04]
        # self.q_pub2 = [0.0,0.0,1.4,0.10]
        # self.q_pub3 = [0.0,0.7,1.4,0.09]
        # self.q_pub4 = [-3.14,-3.14,-1.74,0.10]
        # self.q_pub5 = [0.0,0.0,-0.17,1.74]
        # self.q_pub6 = [-3.14,-3.14,2.96,-1.39]
        # self.q_pub1=[3.14,3.14,-1.74]
        # self.q_pub2 = [0.0,0.0,1.4]
        # self.q_pub3 = [0.0,-0.7,1.4]
        # self.q_pub4 = [3.14,3.14,1.74]
        # self.q_pub5 = [0.0,0.0,0.17]
        # self.q_pub6 = [3.14,3.14,2.96]
        self.round3 = partial(round, ndigits=3)
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_positions = Float64MultiArray()
        self.wheel_velocities = Float64MultiArray()
        # self.cli = self.create_client(SetBool, '/demo/switch')
        # while not self.cli.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('service not available, waiting again...')
        # self.req = SetBool.Request()
    # def send_request(self, status):
    #     self.req.data = status
    #     self.future = self.cli.call_async(self.req)
    #     #rclpy.spin_until_future_complete(self, self.future)
    #     return self.future.result()
    def pick_position(self):
        linear_vel = 0.0
        steer_angle = 0.0
        #pose1
        self.wheel_velocities.data = [linear_vel, -linear_vel, linear_vel, -linear_vel, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_positions.data = [steer_angle, steer_angle, float(-3.14), float(0.0), float(0.0),float(-3.14),float(0.0),float(-3.14)]
        time.sleep(2)
        self.joint_position_pub.publish(self.joint_positions)
        time.sleep(2)
        self.wheel_velocities_pub.publish(self.wheel_velocities)
        #pose2
        self.joint_positions.data = [steer_angle, steer_angle, float(-3.14), float(0.0), float(0.7),float(-3.14),float(0.0),float(-3.14)]
        time.sleep(2)
        self.joint_position_pub.publish(self.joint_positions)
        time.sleep(2)
        self.wheel_velocities_pub.publish(self.wheel_velocities)
        time.sleep(2)
        #pose3
        self.joint_positions.data = [steer_angle, steer_angle, float(1.65), float(1.65), float(1.65),float(-1.48),float(0.102),float(-3.03)]
        self.joint_position_pub.publish(self.joint_positions)
        time.sleep(2)
        self.wheel_velocities_pub.publish(self.wheel_velocities)
        time.sleep(2)
        self.wheel_velocities.data = [linear_vel, -linear_vel, linear_vel, -linear_vel, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_positions.data = [steer_angle, steer_angle, float(0.058), float(0.058), float(0.058),float(0.058),float(-1.5819),float(-1.559)]
        
        self.joint_position_pub.publish(self.joint_positions)
        self.wheel_velocities_pub.publish(self.wheel_velocities)
        # self.joint_positions.data = [steer_angle, steer_angle, float(-3.04), float(0.10), float(0.09),float(0.10),float(1.74),float(-1.39)]
        # self.joint_positions.data = [steer_angle, steer_angle, float(-0.39), float(-0.10), float(0.09),float(0.10),float(1.74),float(-1.39)]
        
        # self.joint_position_pub.publish(self.joint_positions)
        # self.wheel_velocities_pub.publish(self.wheel_velocities)
        # j=0
        # linear_vel = 0.0
        # steer_angle = 0.0
        # while (j<=2):
           
        #     self.wheel_velocities.data = [linear_vel, -linear_vel, linear_vel, -linear_vel, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #     self.joint_positions.data = [steer_angle, steer_angle, float(self.q_pub1[j]), float(self.q_pub2[j]), float(self.q_pub3[j]),float(self.q_pub4[j]),float(self.q_pub5[j]),float(self.q_pub6[j])]
        #     time.sleep(2)
        #     self.joint_position_pub.publish(self.joint_positions)
        #     time.sleep(2)
        #     self.wheel_velocities_pub.publish(self.wheel_velocities)
            
        #     j+=1
    def place_position(self):
        q_publ1=[-3.04]
        q_publ2 = [0.10]
        q_publ3 = [0.09]
        q_publ4 = [0.10]
        q_publ5 = [1.74]
        q_publ6 = [-1.39]
        linear_vel = 0.0
        steer_angle = 0.0
       
        
        # self.wheel_velocities.data = [linear_vel, -linear_vel, linear_vel, -linear_vel, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # self.joint_positions.data = [steer_angle, steer_angle, float(-3.04), float(0.10), float(0.09),float(0.10),float(1.74),float(-1.39)]
        # # self.joint_positions.data = [steer_angle, steer_angle, float(math.pi-3.04), float(-0.10), float(-0.09),float(-0.10),float(1.74),float(-1.39)]
        
        # self.joint_position_pub.publish(self.joint_positions)
        # self.wheel_velocities_pub.publish(self.wheel_velocities)
        time.sleep(0.05)
        
     

   
def main(args=None):
    

    rclpy.init(args=args)
    node = Circle_UR5()
    node.pick_position()
    time.sleep(3)
    # node.place_position()
    # status = True
    # node.send_request(status)
    # print("Gripper Status", response)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        

