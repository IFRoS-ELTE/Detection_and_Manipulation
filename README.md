# Introduction

This module includes:
- Detection code: using a trained YOLO model called best2.pt the code detects tennis balls in the camera images and computes the 3D position of the ball.
- Arm manipulation using resolved motion control algorithm + recursive task priority algorithm to move the 6 dof robotic arm called xarm6.

## Created by:

Amine Dhemaied

Zeljko Jovanovic

Moshin Kabir


# Requirements
ROS

DOCKER

agx_arm6 library (for Gazebo simulation)
## agx_arm6

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
First you need to compile the docker container as shown below:

# Docker

## Building the container
go into the docker folder, then xecute
```
sudo docker compose build
```

## Start the container
go into the docker folder, then execute
```
sudo docker compose up
```

## Stop the docker container
just hit control+C on the CLI where the container was started

## Enter the container instance (when the container is running)
in a new CLI, execute
```
sudo docker exec -it <Container name> bash
```

## Exit the container
In order to exit the container you just need to write in the CLI inside the container
```
exit
```
## Run the code

Once you are in the container you can just go to the src folder and run:
### Detection
#### 'tennis_yolo_real.py' 
to subscribe to the realsense camera on the robot and run the model and compute the 3D position of the detected ball with respect to the end-effector of the arm USING the focal length and ball real size. the code publishes the final 3d pose and and image with detection bounding box and pose written.
OR
#### 'tennis_yolo_real_class_compute.py'  
to subscribe to the realsense camera on the robot and run the model and compute the 3D position of the detected ball with respect to the end-effector of the arm USING depth information. the code publishes the final 3d pose and and image with detection bounding box and pose written.

### Arm Manipulation
#### 'move_to_point_simulation.py' 
to move the arm to a target pose in Gazebo (you just need to edit the goal position named sigma_d in line 16)

#### 'move_to_point_real.py' 
to move the arm to a target pose in the real robot (you just need to edit the goal position named sigma_d in line 34)

#### 'move_to_detected_pose.py' 
to subscribe to the goal position published by the detection code and transform it to the base frame and move the arm.
(For safety reasons lines 77 and 78 are commented, the code now will just print the final pose after transformation. If you want to move the arm to this target position you just need to uncomment both lines)
