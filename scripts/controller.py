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

class PickUpState():
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
        
def send_pose(pose: Pose):
    global current_pose
    current_pose = pose

def send_int_pose(pose: Pose):
    global int_pose
    int_pose = pose

def joint_states_pos(angles: JointState):
    global current_joint_angle
    current_joint_angle = angles

def des_joint_states_pos(angles: JointState):
    global current_des_joint_angle
    current_des_joint_angle = angles

def fid_trans(poses: FiducialTransformArray) -> Pose:
    global fid_length
    try:
        fid_length = len(poses.transforms)
    except Exception as e:
        print(e)

def detect_color(color: ColorRGBA):
    global color_array
    color_array = np.array([0,0,0])
    color_array[0] = color.r
    color_array[1] = color.g
    color_array[2] = color.b

def main():
    # global joint_pub
    # global gripper_pub
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
    rospy.init_node("controller_node")

    # gripper_pub = rospy.Publisher(
    #     'desired_gripper_state', # Topic name
    #     Bool, # Message type
    #     queue_size=10 # Topic size (optional)
    # )

    rate = rospy.Rate(50)

    #state 0
    while True:
        while True:
            for i in range(100):
                IdleState().execute()
            break

        time.sleep(1)

        olderPose = np.array([0,0,0])
        final_pose = Pose()

        #check rotation state
        while True:
            rospy.loginfo('check rotation state')
            global current_pose
            # Send out state instructions
            #waiting for block to stop
            try:
                #get current position
                current_rot_x = current_pose.orientation.x
                current_rot_y = current_pose.orientation.y
                current_rot_z = current_pose.orientation.z

                current_rot_array = np.array([current_rot_x, current_rot_y, current_rot_z])
            except NameError:
                print("name error")

            # wait time to check not moving -- 3 second
            time.sleep(1)

            try:
                # compare old orientation to new orientation
                if ((list(np.array([-0.0005,-0.0005,-0.0005])) <= list(olderPose - current_rot_array) <= list(np.array([0.0005,0.0005,0.0005])))):
                    final_pose.position.x = int_pose.position.x
                    final_pose.position.y = int_pose.position.y
                    final_pose.position.z = int_pose.position.z
                    break
            except NameError:
                print('not defined')
            try:
                olderPose = current_rot_array
            except NameError:
                print("name error")

        #intermediate state
        while True:
            rospy.loginfo('intermediate state')
            # Send out state instructions
            try:
                for i in range(100):
                    pose_pub.publish(final_pose)
                break
            except NameError:
                print('not defined')

        final_pose.position.z = 0.07
        
        time.sleep(3)

        #final state
        final_joint_states = JointState(
            # Set header with current time
            header=Header(stamp=rospy.Time.now()),
            # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
            name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
        )

        #intermediate state again for pick up
        while True:
            rospy.loginfo('int state again')
            # Send out state instructions
            #waiting for block to stop
            # Send out state instructions

            try:
                for i in range(100):
                    pose_pub.publish(final_pose)
                    final_joint_states = current_des_joint_angle
                break
            except NameError:
                print('not defined')

        angle_2 = list(final_joint_states.position)
        angle_2[1] += np.pi/4
        final_joint_states.position = [angle_2[0], angle_2[1], angle_2[2], angle_2[3]]

        time.sleep(4)

        #state 2    
        while True:
            GripState(False).execute()
            break

        time.sleep(2)

        #post pickup state
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

        #state 3    
        while True:
            #pick up
            PickUpState().execute()

            #check pos
            try:
                time.sleep(1)
                #check at position
                if (list(current_des_joint_angle.position - np.array([0.03, 0.03, 0.03, 0.03])) <= list(current_joint_angle.position)[::-1] <= list(current_des_joint_angle.position + np.array([0.03, 0.03, 0.03, 0.03]))):
                    break
            except NameError:
                print('not defined')

        rospy.loginfo("Checking colour")

        time.sleep(3)

        # check colour
        bin_int = JointState(
            # Set header with current time
            header=Header(stamp=rospy.Time.now()),
            # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
            name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
        )

        bin_fin = JointState(
            # Set header with current time
            header=Header(stamp=rospy.Time.now()),
            # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
            name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
        )

        while True:
            #all dark
            if (color_array[0] < 100 and color_array[1] < 100 and color_array[2] < 100):
                break
            rospy.loginfo("Validating next state")
            if (np.argmax(color_array) == 0):
                #execute red state
                rospy.loginfo("Red state executing")
                for i in range(100):
                    HomeState().execute()
                bin_int.position = [2.05,0,-1.32,1.06]
                bin_fin.position = [2.05,-0.73,-1.32,1.06]
                #Intermediate position
                for i in range(100):
                    joint_pub.publish(bin_int)
                time.sleep(2)
                for i in range(100):
                    joint_pub.publish(bin_fin)
                time.sleep(2)
                GripState(True).execute()
                break
            
            if (np.argmax(color_array) == 1):
                #execute green state
                rospy.loginfo("Green state executing")
                for i in range(100):
                    HomeState().execute()
                bin_int.position = [2.6,-0.66,0,1.46]
                bin_fin.position = [2.6,-1.1,-0.59,1.46]
                #Intermediate position
                for i in range(100):
                    joint_pub.publish(bin_int)
                time.sleep(2)
                for i in range(100):
                    joint_pub.publish(bin_fin)
                time.sleep(2)
                GripState(True).execute()
                break

            if (np.argmax(color_array) == 2):
                #execute blue state
                rospy.loginfo("Blue state executing")
                for i in range(100):
                    HomeState().execute()
                bin_int.position = [-2.6,-0.84,0,1.43]
                bin_fin.position = [-2.6,-1.03,-0.59,1.43]
                #Intermediate position
                for i in range(100):
                    joint_pub.publish(bin_int)
                time.sleep(2)
                for i in range(100):
                    joint_pub.publish(bin_fin)
                time.sleep(2)
                GripState(True).execute()
                break
            
            if (np.argmin(color_array) == 2 and (0.95 <= color_array[0]/color_array[1] <= 1.05)):
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