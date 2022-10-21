#!/usr/bin/env python3

# Always need this
from ast import Str
from tkinter import E
import rospy

# Import message types
from smach import State, StateMachine
import smach_ros
from std_msgs.msg import Header, Bool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
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

class GripCloseState():
    def __init__(self):
    # Your state initialization goes here
        # Your state execution goes here
        self.grip_pub = rospy.Publisher(
            'desired_gripper_state', # Topic name
            Bool, # Message type
            queue_size=10 # Topic size (optional) 
        )

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
        gripperState.data = False
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
        self.home_pos = JointState(
            # Set header with current time
            header=Header(stamp=rospy.Time.now()),
            # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
            name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
        )

    def execute(self):
        rospy.loginfo('state 3')
        self.home_pos.position = [0,-0.59,0,0]
        self.joint_pub.publish(self.home_pos)
        
def send_pose(pose: Pose):
    global current_pose
    current_pose = pose

def joint_states_pos(angles: JointState):
    global current_joint_angle
    current_joint_angle = angles

def des_joint_states_pos(angles: JointState):
    global current_des_joint_angle
    current_des_joint_angle = angles

def main():
    global current_pose
    start = True
    # global joint_pub
    # global gripper_pub
    global pose_pub
    global pose_sub
    global joint_states_sub
    global des_joint_states_sub

    pose_sub = rospy.Subscriber(
            'current_pose',
            Pose,
            send_pose
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
    rospy.init_node("controller_node")

    # gripper_pub = rospy.Publisher(
    #     'desired_gripper_state', # Topic name
    #     Bool, # Message type
    #     queue_size=10 # Topic size (optional)
    # )

    rate = rospy.Rate(10)

    #state 0
    while True:
        if (start == True):
            for i in range(100):
                IdleState().execute()
            break

    time.sleep(1)

    #state 1
    start = False

    olderPose = np.array([0,0,0])

    while True:
        rospy.loginfo('state 1')
        # Send out state instructions
        global current_pose
        #waiting for block to stop
        try:
            #get current position
            current_x = current_pose.position.x
            current_y = current_pose.position.y
            current_z = current_pose.position.z

            current_pose_array = np.array([current_x, current_y, current_z])
        except NameError:
            print("name error")

        #wait 1 second
        time.sleep(1)

        try:
            #compare old pos to new pos
            if (list(np.array([-0.0005,-0.0005,-0.0005])) <= list(olderPose - current_pose_array) <= list(np.array([0.0005,0.0005,0.0005]))):
                try:
                    pose_pub.publish(current_pose)
                    time.sleep(2)
                    #check at position
                    if (list(current_des_joint_angle.position - np.array([0.03, 0.03, 0.03, 0.03])) <= list(current_joint_angle.position)[::-1] <= list(current_des_joint_angle.position + np.array([0.03, 0.03, 0.03, 0.03]))):
                        break
                except NameError:
                    print('not defined')
        except NameError:
            print('not defined')

        try:
            olderPose = current_pose_array
        except NameError:
            print("name error")

    time.sleep(1)

    #state 2    
    while True:
        GripCloseState().execute()
        break

    time.sleep(1)

    #state 3    
    while True:
        #pick up
        PickUpState().execute()

        #check pos
        try:
            time.sleep(2)
            #check at position
            if (list(current_des_joint_angle.position - np.array([0.03, 0.03, 0.03, 0.03])) <= list(current_joint_angle.position)[::-1] <= list(current_des_joint_angle.position + np.array([0.03, 0.03, 0.03, 0.03]))):
                break
        except NameError:
            print('not defined')

    time.sleep(1)

    #check colour
    
    
    # with sm:
    #     # Add states to the container
    #     StateMachine.add('IdleState', IdleState(), transitions={'home_config':'SearchState'})
    #     StateMachine.add('SearchState', SearchState(), transitions={'search':'MoveBlock'})
    #     StateMachine.add('MoveBlock', MoveBlock(), transitions={'move_block':'task_done'})

    # sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    # sis.start()

    # # Execute SMACH plan
    # sm.execute()

    # # rospy.spin()
    # sis.stop()
    rospy.spin()


if __name__ == '__main__':
    main()