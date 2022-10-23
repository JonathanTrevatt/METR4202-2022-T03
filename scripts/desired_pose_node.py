#!/usr/bin/env python3
"""
This script publishes a desired pose
"""

#Imports

import rospy
from geometry_msgs.msg import Pose
from fiducial_msgs.msg import FiducialTransformArray
import numpy as np

"""
fid_trans: callback function which reads in fiducial transform array 
and selects a tag to be published to the controller.

Params: poses -> fiducial transform array
"""
def fid_trans(poses: FiducialTransformArray) -> Pose:
    global pose_pub
    #initialise array of tag coordinates in the robot frame
    fiducial_array = []
    #check if there is a tag present
    if (len(poses.transforms) > 0):
        for i in range(len(poses.transforms)):
            if (poses.transforms[i].fiducial_id is not None):
                #get translations and rotations from each element of fiducial transform array
                x = poses.transforms[i].transform.translation.x
                y = poses.transforms[i].transform.translation.y
                z = poses.transforms[i].transform.translation.z

                rot_x = poses.transforms[i].transform.rotation.x
                rot_y = poses.transforms[i].transform.rotation.y
                rot_z = poses.transforms[i].transform.rotation.z

                #assemble a column vector for frame transformation operations
                pc = np.array([[x],[y],[z],[1]])
                rot_array = np.array([rot_x,rot_y,rot_z])
                #define transformation matrix from robot to camera frame
                Trc = np.array([[0,1,0,0.19],[1,0,0,-0.015],[0,0,-1,0.47],[0,0,0,1]])
                #calculate tag position in robot frame
                psb_4 = (Trc @ pc).T
                #join translations and rotations into one vector
                pose_array = np.concatenate((psb_4[:,0:3][0],rot_array),axis=None)
                #append each pose array into the fiducial array
                fiducial_array.append(pose_array)
        #pick random id
        rand_num = np.random.randint(0,len(fiducial_array))

        #publish out selected tag pose
        pose_pub.publish(get_pose(fiducial_array[rand_num]))
        #publish out intermediate pose
        int_pub.publish(get_int_pose(fiducial_array[rand_num]))

    else:
        #ignore
        pass

"""
get_pose: assembles a final desired pose message with position and orientation
Params: psb -> desired position array of type (x,y,z)
Return: Pose message
"""
def get_pose(psb):
    pose_msg = Pose()
    pose_msg.position.x = psb[0]
    pose_msg.position.y = psb[1]
    #set height to 7 cm above base (height of rotating disk which is static)
    pose_msg.position.z = 0.07

    pose_msg.orientation.x = psb[3]
    pose_msg.orientation.y = psb[4]
    pose_msg.orientation.z = psb[5]

    return pose_msg

"""
get_pose: assembles a intermediate pose message with position and orientation
Params: psb -> desired position array of type (x,y,z)
Return: Pose message
"""
def get_int_pose(psb):
    pose_msg = Pose()
    pose_msg.position.x = psb[0]
    pose_msg.position.y = psb[1]
    #set height to 9 cm above base
    pose_msg.position.z = 0.09

    pose_msg.orientation.x = psb[3]
    pose_msg.orientation.y = psb[4]
    pose_msg.orientation.z = psb[5]

    return pose_msg

def main():
    global pose_pub
    global int_pub
    # Initialise node
    rospy.init_node('pose_node')
    # Create subscriber listening to fiducial transforms
    sub = rospy.Subscriber(
        'fiducial_transforms', # Topic name
        FiducialTransformArray, # Message type
        fid_trans # Callback function (required)
    )
    # Create current pose publisher
    pose_pub = rospy.Publisher(
        'current_pose', # Topic name
        Pose, # Message type
        queue_size=10 # Topic size (optional)
    )
    # Create intermediate pose publisher
    int_pub = rospy.Publisher(
        'current_int_pose', # Topic name
        Pose, # Message type
        queue_size=10 # Topic size (optional)
    )

    #set rate to 50 Hz, this is common to all nodes
    rospy.Rate(50)

    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()