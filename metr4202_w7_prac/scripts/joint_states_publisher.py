#!/usr/bin/env python3
"""
This script publishes a set of random joint states to the dynamixel controller.
Use this to get an idea of how to code your inverse kinematics!
"""

# Funny code
import random

# Always need this
import rospy

# Import message types
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose,TransformStamped,Quaternion,Vector3

import modern_robotics as mr
from tf_conversions import transformations as tfct
import numpy as np


# Your inverse kinematics function
# This one doesn't actually do it though...
def inverse_kinematics(pose: Pose) -> JointState:
    global pub
    # TODO: Have fun :)
    Rsb = tfct.quaternion_matrix(np.array([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]))[:3, :3]
    psb = np.array([pose.position.x,pose.position.y,pose.position.z])
    trans_sb = mr.RpToTrans(Rsb, psb)

    M = np.array([[0,-1,0,1],[1,0,0,5],[0,0,1,1],[0,0,0,1]])
    Slist = np.array([[0,1,0,0,0,1],[1,0,0,0,0,-2],[1,0,0,0,0,-3],[1,0,0,0,0,-4]]).T
    thetalist = np.array([0,np.pi/6,np.pi/6,np.pi/6])
    joint_angles = mr.IKinSpace(Slist,M,trans_sb,thetalist,0.1,0.001)
    rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')
    pub.publish(dummy_joint_states(joint_angles))


# Funny code
def dummy_joint_states(joint_angles) -> JointState:
    # Create message of type JointState
    msg = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
    )
    # Funny code
    msg.position = [
        joint_angles[0][0],
        joint_angles[0][1],
        joint_angles[0][2],
        joint_angles[0][3]
    ]
    return msg


def main():
    global pub
    # Create publisher
    pub = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )

    # Create subscriber
    sub = rospy.Subscriber(
        'desired_pose', # Topic name
        Pose, # Message type
        inverse_kinematics # Callback function (required)
    )

    # Initialise node with any node name
    rospy.init_node('joint_node')
    # tfBuffer = tf2_ros.Buffer()
    # listener = tf2_ros.TransformListener(tfBuffer)

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()