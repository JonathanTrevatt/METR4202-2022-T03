# METR4202 Team 03 README File

This file explains how to:

- Launch the robot
- The packages used
- The acknowledgements for third-party code

## Team Members
1) Jared Tucker
2) Akash Iyer
3) Flynn McDermott
4) Chun Yu Ng
5) Jonathan Trevatt

## Usage

Run the following commands in order to build and launch the robot correctly.

### Building and Sourcing
```sh
# Go to workspace source directory
cd ~/catkin_ws
# Build workspace
catkin build
# Source workspace
# (only when adding new packages/scripts, always with new terminals)
source devel/setup.bash
```

### Enabling/Disabling GPIO daemon
```sh
#initialise GPIO daemon
sudo pigpiod
#optional: if want to kill GPIO daemon
sudo killall pigpiod
```

### Disabling USB Memory Limits
You need to run the following command after each boot to disable the USB memory limits
```console
echo 0 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

### Launching the Robot
Run the following command in the terminal
```console
roslaunch metr4202_w7_prac node_launcher.launch
```

### Change Ximea Camera to Run in RGB Mode
Simply press the space bar to toggle between RGB and Mono.

## Packages
### Dynamixel Interface
- Used to control the robot joints
- contains 2 launch files
  - 'dynamixel_interface_controller.launch'
  - 'dynamixel_interface_slider.launch'
- Use dynamixel_interface_controller to launch a file which sets the joint torques, effectively setting up the robot for action.
- Dynamixel slider has been used for testing hard-coded robot positions outside the range of our inverse kinematics function. It consists of a GUI which can vary the joint angles.

### Dynamixel Slider
Contains config and yaml files that set the GUI variables in dynamixel_interface_slider.launch

### metr4202_w7_prac
Package from which robot is developed:
- contains the following items:
  - launch -- contains launch files in standard package format
  - scripts -- contains code that controls the robot
  - CMakeLists.txt -- used for building
  - package.xml -- contains XML that defines name, version and dependencies and exports to a package.
  - README.md

### metr4202_ximea_ros
Package that controls the camera

- Contains the following packages:
  - ximea_ros: handles viewing the camera image and aruco tag detection
  - ximea_color: handles color detection

### vision_opencv
The opencv library that handles computer vision

## Node Information
### dynamixel_interface_controller_node
Handles interfacing with dynamixel motors
### image_node
Handles interfacing with the ximea camera
### controller_node
Handles FSM (Finite State Machine) for robot
### joint_node
Handles publishing of joint angles and inverse kinematics to configure robot
### gripper_node
Handles Raspberry Pi GPIO interfacing for servo grip and release functions
### pose_node
Handles publishing of camera to robot frame transformation
### aruco_detect
Handles detection of aruco tags
### n__ximea_ros
Master camera node
### rosout
Console log reporting mechanism for ROS

## Acknowledgements
We would like to thank all the wonderful tutors teaching METR4202: Josh, Nilp, Ben, Shanker, Miguel and Kenzo. We would especially like to thank Miguel, who has kindly provided the metr4202_ximea_ros package.

We would also like to acknowledge Kenji Brameld (Github tag @ijnek) for usage of the vision_opencv library.