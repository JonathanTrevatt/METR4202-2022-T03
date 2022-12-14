#!/usr/bin/env python3

# Always need this
import rospy

# Import message types
from std_msgs.msg import Header, Bool, ColorRGBA
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from fiducial_msgs.msg import FiducialTransformArray
import numpy as np

import time

global current_pose

"""
IdleState: a class that configures robot to home position with gripper open
"""
class IdleState():
    def __init__(self):
    # Your state initialization goes here
        # Your state execution goes here
        self.joint_pub = rospy.Publisher(
            'desired_joint_states', # Topic name
            JointState, # Message type
            queue_size=10 # Topic size (optional)    
        )
        self.grip_pub = rospy.Publisher(
            'desired_gripper_state', # Topic name
            Bool, # Message type
            queue_size=10 # Topic size (optional) 
        )
        self.home_pos = JointState(
            # Set header with current time
            header=Header(stamp=rospy.Time.now()),
            # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
            name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
        )

    def execute(self):
        rospy.loginfo('state 0')
        self.home_pos.position = [0,0,0,0]
        self.joint_pub.publish(self.home_pos)
        gripperState = Bool()
        gripperState.data = True
        self.grip_pub.publish(gripperState)

"""
GripState: a class that closes or opens the gripper
"""
class GripState():
    def __init__(self, boolean):
    # Your state initialization goes here
        # Your state execution goes here
        self.grip_pub = rospy.Publisher(
            'desired_gripper_state', # Topic name
            Bool, # Message type
            queue_size=10 # Topic size (optional) 
        )

        self.boolean = boolean

    def execute(self):
        rospy.loginfo('state 2')

        # self.time_init = time.time()
        # self.time_out = self.time_init + 5
        # while True:
        #     if (self.time_init > self.time_out):
        #         break
        #     else:
                # Send out state instructionss)
        gripperState = Bool()
        #closed
        gripperState.data = self.boolean
        self.grip_pub.publish(gripperState)

"""
ShowCameraState: a class that executes 'pick up' of cube and shows it to camera for colour detection
"""
class ShowCameraState():
    def __init__(self):
    # Your state initialization goes here
        # Your state execution goes here
        self.joint_pub = rospy.Publisher(
            'desired_joint_states', # Topic name
            JointState, # Message type
            queue_size=10 # Topic size (optional)    
        )
        self.pick_pos = JointState(
            # Set header with current time
            header=Header(stamp=rospy.Time.now()),
            # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
            name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
        )

    def execute(self):
        rospy.loginfo('state 3')
        self.pick_pos.position = [-0.07, -0.88, -0.04, -0.91]
        self.joint_pub.publish(self.pick_pos)

"""
HomeState: a class that returns home position
"""
class HomeState():
    def __init__(self):
    # Your state initialization goes here
        # Your state execution goes here
        self.joint_pub = rospy.Publisher(
            'desired_joint_states', # Topic name
            JointState, # Message type
            queue_size=10 # Topic size (optional)    
        )
        self.home_pos = JointState(
            # Set header with current time
            header=Header(stamp=rospy.Time.now()),
            # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
            name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
        )

    def execute(self):
        rospy.loginfo('state 0')
        self.home_pos.position = [0,0,0,0]
        self.joint_pub.publish(self.home_pos)

"""
send_pose: callback function that updates global variable corresponding to tag's current location

Params: pose -> pose message of tag's current position
"""
def send_pose(pose: Pose):
    global current_pose
    current_pose = pose

"""
send_int_pose: callback function that updates global variable corresponding to where the robot will go in intermediate state
based on tag's current location

Params: pose -> pose message of tag's current position
"""
def send_int_pose(pose: Pose):
    global int_pose
    int_pose = pose

"""
joint_states_pos: callback function that updates global variable corresponding to current joint angle

Params: angles -> JointState message published by dynamixel interface controller
"""
def joint_states_pos(angles: JointState):
    global current_joint_angle
    current_joint_angle = angles

"""
des_joint_states_pos: callback function that updates global variable corresponding to desired joint angle

Params: angles -> JointState message published by joint states publisher
"""
def des_joint_states_pos(angles: JointState):
    global current_des_joint_angle
    current_des_joint_angle = angles

"""
fid_trans: callback function which reads in fiducial transform array 
and updates global variable corresponding to number of tags detected by the camera.

Params: poses -> fiducial transform array
"""
def fid_trans(poses: FiducialTransformArray) -> Pose:
    global fid_length
    try:
        fid_length = len(poses.transforms)
    except Exception as e:
        print(e)

"""
detect_color: callback function which reads in colors published by /test_color and updates global variable.

Params: color -> an RGBA array
"""
def detect_color(color: ColorRGBA):
    global color_array
    color_array = np.array([0,0,0])
    color_array[0] = color.r
    color_array[1] = color.g
    color_array[2] = color.b

"""
main function:
Disclaimer: try and except cases are used to handle instances when callbacks functions have not run yet to define global variables
"""
def main():
    global fid_sub
    global int_pose_sub
    global pose_pub
    global pose_sub
    global joint_states_sub
    global des_joint_states_sub
    global color_sub

    fid_sub = rospy.Subscriber(
        'fiducial_transforms', # Topic name
        FiducialTransformArray, # Message type
        fid_trans # Callback function (required)
    )

    pose_sub = rospy.Subscriber(
        'current_pose',
        Pose,
        send_pose
    )

    int_pose_sub = rospy.Subscriber(
        'current_int_pose',
        Pose,
        send_int_pose
    )

    pose_pub = rospy.Publisher(
        'desired_pose',
        Pose,
        queue_size=10
    )

    joint_states_sub = rospy.Subscriber(
        'joint_states', # Topic name
        JointState, # Message type
        joint_states_pos # Topic size (optional)    
    )

    des_joint_states_sub = rospy.Subscriber(
        'desired_joint_states', # Topic name
        JointState, # Message type
        des_joint_states_pos # Topic size (optional)    
    )
    
    joint_pub = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)    
    )

    color_sub = rospy.Subscriber(
        'test_color',
        ColorRGBA,
        detect_color
    )

    #Initialise node
    rospy.init_node("controller_node")

    #set rate to 50 Hz, this is common to all nodes
    rate = rospy.Rate(50)

    #keep executing states one by one
    while True:
        #first state --> execute idle state (home position with gripper open)
        while True:
            #publish message hundred times to ensure that subscriber receives
            for i in range(100):
                IdleState().execute()
            break
        
        #wait before executing next state
        time.sleep(1)

        #initialise older pose -- this is used to store previous instance of pose for rotation check purposes
        olderPose = np.array([0,0,0])
        #initialise final pose -- position that the cube is picked up at
        final_pose = Pose()

        #state that checks cubes are rotating
        while True:
            rospy.loginfo('check rotation state')
            global current_pose
            #waiting for block to stop rotating
            try:
                #get current orientation
                current_rot_x = current_pose.orientation.x
                current_rot_y = current_pose.orientation.y
                current_rot_z = current_pose.orientation.z

                current_rot_array = np.array([current_rot_x, current_rot_y, current_rot_z])
            except NameError:
                print("name error")

            # wait time to check not moving -- 1 second
            time.sleep(1)

            try:
                # compare old orientation to new orientation
                # if tag is present in scene and orientation within +-0.0005 tolerance, then we know cube has stopped rotating
                if ((list(np.array([-0.0005,-0.0005,-0.0005])) <= list(olderPose - current_rot_array) <= list(np.array([0.0005,0.0005,0.0005]))) and fid_length > 0):
                    #set final pose position to pick up block at the instant that it stops rotating
                    final_pose.position.x = int_pose.position.x
                    final_pose.position.y = int_pose.position.y
                    final_pose.position.z = int_pose.position.z
                    break
            except NameError:
                print('not defined')
            try:
                #store previous pose
                olderPose = current_rot_array
            except NameError:
                print("name error")

        # intermediate state --> hover above block to allow end-effector to be in right position before finally grabbing block
        while True:
            rospy.loginfo('intermediate state')
            #publish the final pose set earlier
            try:
                for i in range(100):
                    pose_pub.publish(final_pose)
                break
            except NameError:
                print('not defined')

        #update final pose to be at height 7 cm above base (final grabbing position)
        final_pose.position.z = 0.07
        
        time.sleep(2)

        #create final state message instance
        final_joint_states = JointState(
            # Set header with current time
            header=Header(stamp=rospy.Time.now()),
            # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
            name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
        )

        #pick up state --> publishes location where cube will be gripped
        while True:

            try:
                for i in range(100):
                    pose_pub.publish(final_pose)
                    final_joint_states = current_des_joint_angle
                break
            except NameError:
                print('not defined')

        #set joint angles for post pick up state
        angle_2 = list(final_joint_states.position)
        angle_2[1] += np.pi/4
        final_joint_states.position = [angle_2[0], angle_2[1], angle_2[2], angle_2[3]]

        time.sleep(2)

        #grip the cube    
        while True:
            GripState(False).execute()
            break

        time.sleep(1)

        #post pick up state which lifts the cube up preventing contact with the ground
        # set final pose back to 12 cm above base
        final_pose.position.z = 0.12
        while True:
            # Send out state instructions
            try:
                for i in range(100):
                    joint_pub.publish(final_joint_states)
                break
            except NameError:
                print('not defined')
        
        time.sleep(1)

        #state that shows the cube to camera for colour detection    
        while True:
            #set angles to try and show the cube facing directly at the camera
            ShowCameraState().execute()

            #check that we've reached desired joint angles
            try:
                time.sleep(1)
                #check at position
                if (list(current_des_joint_angle.position - np.array([0.03, 0.03, 0.03, 0.03])) <= list(current_joint_angle.position)[::-1] <= list(current_des_joint_angle.position + np.array([0.03, 0.03, 0.03, 0.03]))):
                    break
            except NameError:
                print('not defined')

        rospy.loginfo("Checking colour")

        time.sleep(2)

        #initialise intermediate position above bin
        bin_int = JointState(
            # Set header with current time
            header=Header(stamp=rospy.Time.now()),
            # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
            name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
        )

        #initialise final position at appropriate bin
        bin_fin = JointState(
            # Set header with current time
            header=Header(stamp=rospy.Time.now()),
            # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
            name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
        )

        #define random samples of multiple shades for each colour
        # VERY rudimentary machine learning 

        #yellow rgbs
        r_y = [255, 219, 185, 243, 118]
        g_y = [255, 188, 154, 235, 91]
        b_y = [39,38,48,66,36]

        #red rgbs
        r_r = [255, 241, 231, 186, 111]
        g_r = [97, 74, 57, 137, 24]
        b_r = [110,94,54,155,31]

        #green rgbs
        r_g = [129, 87, 78, 43, 115]
        g_g = [255, 185, 149, 191, 41]
        b_g = [65,73,66,68,41]

        #blue rgbs
        r_b = [60, 59, 20, 85, 12]
        g_b = [166, 158, 105, 194, 52]
        b_b = [255,222,216,156,83]

        # state that checks colour and moves to the appropriate bin
        while True:
            #check if no cube has been grabbed (colour of gripper links which are dark purple)
            if (color_array[0] < 100 and color_array[1] < 100 and color_array[2] < 100):
                break
            rospy.loginfo("Validating next state")

            # check that RGB is between minimum and maximum of random sample arrays defined earlier
            # check if the colour is red
            if (np.min(r_r) <= color_array[0] <= np.max(r_r) and 
            np.min(g_r) <= color_array[1] <= np.max(g_r) and 
            np.min(b_r) <= color_array[2] <= np.max(b_r)):
                #execute red state
                rospy.loginfo("Red state executing")
                for i in range(100):
                    HomeState().execute()
                #define joint angles of intermediate configuration above bin
                bin_int.position = [2.05,0,-1.32,1.06]
                #define joint angles of final configuration at bin
                bin_fin.position = [2.05,-0.73,-1.32,1.06]

                #Intermediate position
                for i in range(100):
                    joint_pub.publish(bin_int)
                time.sleep(2)
                #final position
                for i in range(100):
                    joint_pub.publish(bin_fin)
                time.sleep(2)
                #release the cube
                GripState(True).execute()
                break
            
            # check if the colour is green
            if (np.min(r_g) <= color_array[0] <= np.max(r_g) and 
            np.min(g_g) <= color_array[1] <= np.max(g_g) and 
            np.min(b_g) <= color_array[2] <= np.max(b_g)):
                #execute green state
                rospy.loginfo("Green state executing")
                for i in range(100):
                    HomeState().execute()
                bin_int.position = [-0.44, 0, 1.03, -0.8]
                bin_fin.position = [-0.44, 0.59, 1.68, -0.8]
                #Intermediate position
                for i in range(100):
                    joint_pub.publish(bin_int)
                time.sleep(2)
                for i in range(100):
                    joint_pub.publish(bin_fin)
                time.sleep(2)
                GripState(True).execute()
                break

            # check if the colour is blue
            if (np.min(r_b) <= color_array[0] <= np.max(r_b) and 
            np.min(g_b) <= color_array[1] <= np.max(g_b) and 
            np.min(b_b) <= color_array[2] <= np.max(b_b)):
                #execute blue state
                rospy.loginfo("Blue state executing")
                for i in range(100):
                    HomeState().execute()
                bin_int.position = [0.18, 0, 1.03, -0.8]
                bin_fin.position = [0.18, -0.59, 1.68, -0.8]
                #Intermediate position
                for i in range(100):
                    joint_pub.publish(bin_int)
                time.sleep(2)
                for i in range(100):
                    joint_pub.publish(bin_fin)
                time.sleep(2)
                GripState(True).execute()
                break
            
            # check if the colour is yellow (mix of red and green)
            if (np.min(r_y) <= color_array[0] <= np.max(r_y) and 
            np.min(g_y) <= color_array[1] <= np.max(g_y) and 
            np.min(b_y) <= color_array[2] <= np.max(b_y)):
                #execute yellow state
                rospy.loginfo("Yellow state executing")
                for i in range(100):
                    HomeState().execute()
                bin_int.position = [-2.05,0,-1.32,1.06]
                bin_fin.position = [-2.05,-0.73,-1.32,1.06]
                #Intermediate position
                for i in range(100):
                    joint_pub.publish(bin_int)
                time.sleep(2)
                for i in range(100):
                    joint_pub.publish(bin_fin)
                time.sleep(2)
                GripState(True).execute()
                break

            else:
                break

        #reset final_pose so robot does not go back to pick up blocks
        final_pose.position.x = 10
        final_pose.position.y = 10
        final_pose.position.z = 10


if __name__ == '__main__':
    main()