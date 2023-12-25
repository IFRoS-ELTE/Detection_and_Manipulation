# agx_arm6
This repository contains resolved motion control algorithm + recursive task priority algorithm to move the 6 dof robotic arm called xarm6.

In order to run it, first you need to have the xarm_ros repository installed in your catkin_ws by following the below instructions:

```bash
cd ~/catkin_ws/src
git clone https://github.com/xArm-Developer/xarm_ros.git
cd ~/catkin_ws 
catkin_make
```

You have to also clone this repository in your catkin_ws:

```bash
cd ~/catkin_ws/src
git pull https://github.com/AmineDh98/agx_arm6.git
cd ~/catkin_ws 
catkin_make
```

To run the simulation in Gazebo and Rviz:

```bash
roslaunch xarm_gazebo xarm6_beside_table.launch

roslaunch xarm6_moveit_config xarm6_moveit_gazebo.launch

```

to run the simulation with the Gripper added:

```bash
roslaunch xarm6_gripper_moveit_config xarm6_gripper_moveit_gazebo.launch

roslaunch xarm_gazebo xarm6_beside_table.launch add_gripper:=true
```

on the real robot:

```bash
roslaunch agx_xarm_bringup setup_arm.launch
```


for the real robot:
```bash
roslaunch agx_xarm_bringup setup_arm.launch
roslaunch realsense2_camera rs_d435_camera_with_model.launch
```
Now you are good to test the code
exercice2_ros.py is a simple move to goal using resolved motion control algorithm
exercice4_ros.py includes recursive task priority algorithm.


