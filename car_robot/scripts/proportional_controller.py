#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from matplotlib import pyplot as plt
from tf2_ros import TransformBroadcaster, TransformStamped
from threading import Thread
from matplotlib import pyplot as plt
from tf_transformations import euler_from_quaternion
# import tf
import math
#Define Lists
Time = []
Ctrl_angle = []
Error_phi = []
control_x=[]
control_y=[]
posx=[]
posy=[]
final_x=10
final_y=10
class proportionalcontroller(Node):
    def __init__(self):
        super().__init__('proportional_control')
        self.get_logger().info(f'Our proportional controller is now activated...') 
    
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,depth=10)
        # self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10) #Create a publisher to the wheel velocoty controller
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10) #Create a publisher to the joint position controller
        
        self.imu_sub = self.create_subscription(Imu, 'imu_plugin/out', self.imu_callback, qos_profile) #Create a subscriber to the 'imu_plugin/out' topic
        self.imu_sub
        self.position_x = 0.0
        self.position_y = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.robot_inst_heading=0.0
        
        self.phi_robot=0.0
       
        
    def imu_callback(self, msg): 
        #small time delat t
        # self.imu_timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec)/(1000000000) #Grab the Time of record.
        t_del = 0.008
        self.vel_x += msg.linear_acceleration.x * t_del
        self.vel_y += msg.linear_acceleration.y * t_del
        self.position_x += self.vel_x * t_del
        self.position_y += self.vel_y * t_del
        #quart to euler 
        quarternion_pose= msg.orientation
        # timestamp = self.imu_timestamp #Update the node's timestamp
        Quart_list = [quarternion_pose.x, quarternion_pose.y, quarternion_pose.z, quarternion_pose.w]
        _, _, self.phi_robot = euler_from_quaternion(Quart_list)

    def get_ERROR(self):
        err_1 = final_x - self.position_x
        err_2 = final_y - self.position_y

        return err_1, err_2

    def Prop_controller(self, err_1, err_2):
        K_x = 0.54
        K_y = 0.58
        K_phi = 0.392

        car_velocty = math.sqrt((K_x * err_1)**2 + (K_y * err_2)**2)

        robot_inst_heading = self.phi_robot
        final_heading = math.atan2(err_1, err_2)
        Steer_err = final_heading - robot_inst_heading
        steer_angle = K_phi * Steer_err
        Error_phi.append(Steer_err)

        max_steer_angle = 0.3
        min_steer_angle = -0.3
        steer_angle = -max(min(steer_angle, max_steer_angle), min_steer_angle)

        return car_velocty, steer_angle

    def pub(self):
        while rclpy.ok():
            err_1, err_2 = self.get_ERROR()
            linear_vel, steer_angle = self.Prop_controller(err_1, err_2)
            #initialize messages
            wheel_velocities = Float64MultiArray()
            joint_positions = Float64MultiArray()
            new_goal= math.sqrt((final_x-self.position_x)**2 + (final_y-self.position_y)**2)
            self.get_logger().info(f'goal.{new_goal}') 
            if new_goal<2.9:
                # car_velocty=0
                # linear_vel=0
                wheel_velocities.data = [0.0,0.0, 0.0,0.0]
                joint_positions.data = [0.0,0.0]
                self.joint_position_pub.publish(joint_positions)
                self.wheel_velocities_pub.publish(wheel_velocities)
                break
          
            wheel_velocities.data = [linear_vel,-linear_vel, linear_vel,-linear_vel]
            joint_positions.data = [steer_angle,steer_angle]

            self.joint_position_pub.publish(joint_positions)
            self.wheel_velocities_pub.publish(wheel_velocities)
            

def main(args=None):
    print('Starting Proportional Control: Moving from (0,0) to (10,10)') #Confirmation that Main Functions Properly
    rclpy.init(args=args)

    Controller = proportionalcontroller() #Establish the Publisher Node
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(Controller)
    mythread = Thread(target=Controller.pub)
    mythread.start()
    executor.spin()
    mythread.join()

    rclpy.shutdown()
    # Graphs
    # plt.title('Yaw Error vs. Time')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Yaw Error (rad)')
    # plt.plot(Time, Error_phi, 'b-', label = 'Yaw Error')
    # plt.legend()
    # plt.savefig("/home/mayank/testWS/src/car_robot/results/Yaw_error.png")
    # # plt.show()

    # plt.title('Steer Angle vs. Time')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Steer Angle (rad)')
    # plt.plot(Time, Ctrl_angle, 'b-', label = 'Steer Angle')
    # plt.legend()
    
    # plt.savefig("/home/mayank/testWS/src/car_robot/results/Steer_ANgle vs time.png")
    
    # plt.show()

if __name__ == '__main__':
    main()