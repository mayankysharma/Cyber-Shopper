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
from geometry_msgs.msg import Pose2D
# import tf
import math
#Define Lists
Time = []
control = []
error = []
posx=[]
posy=[]
desired_position = Pose2D()
desired_position.x = 10.0
desired_position.y = 10.0


class proportionalcontroller(Node):
    def __init__(self):
        super().__init__('proportional_control')
        # create a subscription to /cmd_vel topic
        self.get_logger().info(f'Our proportional controller is now activated...') 
        self.position_x = 0
        self.position_y = 0
        self.vel_x = 0.0
        self.vel_y = 0.0
 
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST,depth=10)
        # self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.wheel_velocities_pub = self.create_publisher(Float64MultiArray, '/velocity_controller/commands', 10) #Create a publisher to the wheel velocoty controller
        self.joint_position_pub = self.create_publisher(Float64MultiArray, '/position_controller/commands', 10) #Create a publisher to the joint position controller

        self.imu_sub = self.create_subscription(Imu, 'imu_plugin/out', self.imu_callback, qos_profile) #Create a subscriber to the 'imu_plugin/out' topic
        self.imu_sub  # Prevent unused variable warning

        timer_period = 0.1 #Pub Frequency
        self.timer = self.create_timer(timer_period, self.timer_callback) #Initialize Timer Callback

        self.counter = 0 #Initialize Counter
     
    def imu_callback(self, msg): #Define the Callback Function
        self.time_starts = self.get_clock().now()
        imu_timestamp = msg.header.stamp.sec + (msg.header.stamp.nanosec)/(1000000000) #Grab the Time of record.
        current_heading = msg.orientation.z #Grab the robot's current yaw (orientation).
        self.timestamp = imu_timestamp #Update the node's timestamp.
        self.current_heading = current_heading #Update the robot's current heading.
        time_rightnow = self.get_clock().now()
        t_elapsed = (time_rightnow-self.time_starts).nanoseconds/1e9
        self.vel_x+=msg.linear_acceleration.x*0.01
        self.vel_y+=msg.linear_acceleration.y*0.01
        self.position_x+=self.vel_x*0.01
        self.position_y+=self.vel_y*0.01
        # self.get_logger().info(f'Current postion x: {self.position_x},Current postion x:{self.position_y}')

    def timer_callback(self): #Defining the Callback Function
        K = 10 #Proportional Controller Gain

        # des_phi = np.deg2rad(45) #Desired phi, but changed due to IMU data.
        des_phi = 0.375 #Adjusted for IMU Errors
        des_x=10
        des_y=10
        linear_vel=3.0 #Constant Velocity Profile
        steer_angle=0.0 #Initialize steer angle to 0 rad.
        x_error=des_x-self.position_x
        y_error=des_y-self.position_y
        phi_error = des_phi - self.current_heading #Calculate the error between current heading and desired heading.
        steer_angle = -K*phi_error #Generate Control Input
        Position_x=-K*x_error
        Position_y=-K*y_error
        #Implement joint limits because an error is thrown even with the XACRO file limits.
                    #Initialize Messages
        wheel_velocities = Float64MultiArray()
        joint_positions = Float64MultiArray()         
        
        if steer_angle>1.0:
            steer_angle=1.0

        if steer_angle<-1.0:
            steer_angle=-1.0
        curr=int(self.position_x)
        print(curr)
        if  curr>8:
            wheel_velocities.data = [0.0,0.0, 0.0,0.0]
            joint_positions.data = [0.0,0.0]  
        else:
            wheel_velocities.data = [linear_vel,-linear_vel, linear_vel,-linear_vel]
            joint_positions.data = [steer_angle,steer_angle]
        # elif Position_x == 9.0:
        #     # wheel_velocities.data = [linear_vel,-linear_vel, linear_vel,-linear_vel]
        #     # joint_positions.data = [steer_angle,steer_angle]
        #     wheel_velocities.data = [0.0,0.0, 0.0,0.0]
        #     joint_positions.data = [0.0,0.0]
            
            



        
        #Implement proper stop command based on counts. More detail in report.
        # if self.counter < 638:

        # if int(self.position_x )<10.0:
        #     wheel_velocities.data = [linear_vel,-linear_vel, linear_vel,-linear_vel]
        #     joint_positions.data = [steer_angle,steer_angle]
          
            
     

        # else:
        #     wheel_velocities.data = [0.0,0.0, 0.0,0.0]
        #     joint_positions.data = [0.0,0.0]
            # for velocity in wheel_velocities.data:
            #     # Calculate the distance for each wheel
            #     initial_distance = velocity * Time
            #     distance += initial_distance



        #Publish joint and velocity commands.
        self.joint_position_pub.publish(joint_positions)
        self.wheel_velocities_pub.publish(wheel_velocities)

        #Terminal output.
        self.get_logger().info(f'Current Heading (Subscription): {self.current_heading}, Steer Control Input (Publisher): {steer_angle}')

        #Append data for plotting.
        Time.append(self.timestamp)
        control.append(steer_angle)
        error.append(phi_error)
        # posx.append(self.position_x)
        # posy.append(self.position_y)
        # Integrate acceleration to update velocity
     

        # Integrate velocity to update position
 

        self.counter += 1 #Increase Counter




def main(args = None): #Defining the 'Main' Function
    print('Starting Proportional Control: Moving from (0,0) to (10,10)') #Confirmation that Main Functions Properly
    rclpy.init(args=args)

    Controller = proportionalcontroller() #Establish the Publisher Node

    try:
        rclpy.spin(Controller) #Spin the Publisher Node
    except KeyboardInterrupt:
        pass

    Controller.destroy_node() #Destroy node when Ctrl. C is pressed
    rclpy.shutdown() #Shut the Node Down

    #After Ctl. C, Plot the Results.
    print('End of monitoring: prepare for graphs!')
    


    ##-------------------Plotting Results---------------------------##
    plt.title('Yaw Error vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw Error (rad)')
    plt.plot(Time, error, 'b-', label = 'Yaw Error')
    plt.legend()
    plt.show()

    plt.title('Steer Angle vs. Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Steer Angle (rad)')
    plt.plot(Time, control, 'b-', label = 'Steer Angle')
    plt.legend()
    plt.show()
    ## position
    # plt.title('pos vs. Time')
    # plt.xlabel('Time (s)')
    # plt.ylabel('Steer Angle (rad)')
    # plt.plot(posx, posy, 'b-', label = 'Steer Angle')
    # plt.legend()
    # plt.show()


if __name__ == '__main__':
    main()