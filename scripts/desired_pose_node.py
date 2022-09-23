#!/usr/bin/env python3
"""
This script publishes a desired pose to the metr4202 prac wk7 node.
"""

import rospy

from geometry_msgs.msg import Pose,TransformStamped,Quaternion,Vector3

import modern_robotics as mr
import tf2_ros
from tf_conversions import transformations as tfct
import numpy as np
from std_msgs.msg import Float32

def main():
    global pub
    # Initialise node with any node name
    rospy.init_node('pose_node')
    # Create publisher
    pose_pub = rospy.Publisher(
        'desired_pose', # Topic name
        Pose, # Message type
        queue_size=10 # Topic size (optional)
    )
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            Rsb = np.array([[0,-1,0],
            [1,0,0],
            [0,0,1]])
            psb = np.array([0.25,0,0.0])
            T_sb = mr.RpToTrans(Rsb,psb)
            q = tfct.quaternion_from_matrix(T_sb)
            
            pose_msg = Pose()
            pose_msg.position.x = psb[0]
            pose_msg.position.y = psb[1]
            pose_msg.position.z = psb[2]

            pose_msg.orientation.x = q[0]
            pose_msg.orientation.y = q[1]
            pose_msg.orientation.z = q[2]
            pose_msg.orientation.w = q[3]

            pose_pub.publish(pose_msg)
            rospy.sleep(2)
        except rospy.ROSException:
            print('Error message not published!')

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()
