# ENPM 662: Project 2

## Project title: Cyber Shopper

### Dependencies:
* sympy
* math
* numpy
* sys
* ROS Controller package and controller manager
```bash
sudo apt install ros-galactic-ros2-control ros-galactic-ros2-controllers ros-galactic-gazebo-ros2-control
sudo apt-get install ros-galactic-controller-manager
```
* Make sure that you start the controllers after spawning the robot
```bash
ros2 control load_controller --set-state start joint_state_broadcaster

ros2 control load_controller --set-state start velocity_controller

ros2 control load_controller --set-state start position_controller
```
### Steps to execute the project:
* Create a ROS2 workspace.

* git clone the package in your ros2 workspace.:
```bash
cd ros2_ws/src
git clone "link to repository"
```

* Source your bashrc and ROS2 version, build your package, and Source your workspace. 
```bash
cd ros2_ws/
source /opt/ros/galactic/setup.bash
colcon build
source install/setup.bash
```

- To spawn the model in empty world for teleop:
```bash 
source ~/workspace/install/setup.bash
ros2 launch cyber_shopper gazebo.launch.py
```
Then open second terminal:-
```bash
source ~/workspace/install/setup.bash
ros2 run cyber_shopper teleoperation.py
```

- To visualize model in RViz with joint sliders:
```bash
ros2 launch cyber_shopper display.launch.py
```

- To visualize the inverse kinematic validation in gazebo:
```bash
source ~/workspace/install/setup.bash
ros2 launch cyber_shopper gazebo.launch.py
```
In the second terminal, run the following command:
```bash
ros2 run cyber_shopper circle_new.py
```
* UR5_Kinematics.ipynb file was used to perform Forward Kinematics and Inverse Kinematics
* CAD files in two folders named: CAD Assembly files 
* To run the proportional controller make sure the position and orientation values in spawn_robot_ros2.launch.py are 0. For running the proportional controller
```bash
source WS_DIRECTORY/install/setup.bash
ros2 launch cyber_shopper gazebo.launch.py
ros2 run cyber_shopper proportional_controller.py
```
### Results:
- [RViZ visualization](https://youtu.be/cSio3TE0TwA)

- [Inverse kinematics validation video]( https://youtu.be/m0AeLGM4_60)

- [Pick n Place operation](https://youtu.be/khTUD25mJzs)
-  [Porportional Controller](https://drive.google.com/file/d/1BP5SyhMt2736mu-GqzJc1gzScRwpLrSq/view?usp=drive_link) 

