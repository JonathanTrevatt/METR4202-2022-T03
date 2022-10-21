#!/usr/bin/env python3
"""
This script publishes a desired pose to the metr4202 prac wk7 node.
"""

import rospy

from geometry_msgs.msg import Pose,TransformStamped,Quaternion,Vector3

import modern_robotics as mr
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray
import numpy as np
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, Quaternion,Vector3, Transform

def fid_trans(poses: FiducialTransformArray) -> Pose:
    global pose_pub
    fiducial_array = []
    if (len(poses.transforms) > 0):
        for i in range(len(poses.transforms)):
            if (poses.transforms[i].fiducial_id is not None):
                x = poses.transforms[i].transform.translation.x
                y = poses.transforms[i].transform.translation.y
                z = poses.transforms[i].transform.translation.z

                pc = np.array([[x],[y],[z],[1]])
                Trc = np.array([[0,1,0,0.19],[1,0,0,-0.015],[0,0,-1,0.47],[0,0,0,1]])
                psb_4 = (Trc @ pc).T
                psb = psb_4[:,0:3][0]
                fiducial_array.append(psb)
                pose_pub.publish(get_pose(fiducial_array[0]))

def get_pose(psb):
    pose_msg = Pose()
    pose_msg.position.x = psb[0]
    pose_msg.position.y = psb[1]
    pose_msg.position.z = 0.07

    return pose_msg

def main():
    global pose_pub
    # Initialise node with any node name
    rospy.init_node('pose_node')
    # Create subscriber
    sub = rospy.Subscriber(
        'fiducial_transforms', # Topic name
        FiducialTransformArray, # Message type
        fid_trans # Callback function (required)
    )
    # Create publisher
    pose_pub = rospy.Publisher(
        'current_pose', # Topic name
        Pose, # Message type
        queue_size=10 # Topic size (optional)
    )

    

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()