#!/usr/bin/env python3
"""
This script publishes a set of random joint states to the dynamixel controller.
Use this to get an idea of how to code your inverse kinematics!
"""

# Funny code
import math

# Always need this
import rospy

# Import message types
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose,TransformStamped,Quaternion,Vector3

import modern_robotics as mr
from tf_conversions import transformations as tfct
import numpy as np
import math


# Your inverse kinematics function
# This one doesn't actually do it though...
def inverse_kinematics(pose: Pose) -> JointState:
    global pub
    # TODO: Have fun :)
    Rsb = tfct.quaternion_matrix(np.array([pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]))[:3, :3]
    psb = np.array([pose.position.x,pose.position.y,pose.position.z])
    trans_sb = mr.RpToTrans(Rsb, psb)

    # Input Arm Link Lengths (meters)
    # L1 = 0.0145
    # L2 = 0.021
    # L3 = 0.052
    # L4 = 0.1175
    # L5 = 0.095
    # L6 = 0.09761
    # L7 = 0.015

    #Links Lengths in meters
    #Links Lengths in meters
    L4 = 0.1175
    L5 = 0.095
    L6 = 0.09761

    # Desired Coordinates
    xd = 1
    yd = -0.1
    zd = 0
    phi = 0

    # Translated coordinates to radius and height
    rad_des = math.sqrt((math.pow(xd,2)+math.pow(yd,2)))
    z_des = zd

    #Make 2R offset changes
    rad = rad_des - L6*math.cos(phi)
    z = z_des - L6*math.sin(phi)
    rad_2r = math.sqrt((math.pow(rad,2)+math.pow(z,2)))

    if ((L4+L5) >= rad_2r):
        theta1 = math.atan2(yd,xd)

        #Elbow up
        sigma = -1


        ctheta3 = (math.pow(rad,2)+math.pow(z,2)-math.pow(L4,2)-math.pow(L5,2))/(2*L4*L5)

        #theta3 = sigma*math.acos(ctheta3)
        theta3 = math.atan2((sigma*math.sqrt((1-math.pow(ctheta3,2)))),ctheta3)  # math.atan2 passes y first then x as arguements, may need to switch

        theta2 = math.atan2(z,rad) - math.atan2((L5*math.sin(theta3)),(L4+L5*math.cos(theta3)))

        theta4 = phi-(theta2+theta3)

        print(theta1,theta2,theta3,theta4)

        if ((-2.3 <= (theta2 - np.pi/2) <= 2.3) and 
        (-2.56 <= theta3 <= 2.56) and
        (-1.76 <= theta4 <= 1.76)):
            print('loop 1')
            joint_angles = [theta1, (theta2 - np.pi/2), theta3, -theta4]

        else:
            print('loop 2')
            #Elbow down variable
            sigma = 1

            ctheta3 = (math.pow(rad,2)+math.pow(z,2)-math.pow(L4,2)-math.pow(L5,2))/(2*L4*L5)

            #theta3 = sigma*math.acos(ctheta3)
            theta3 = math.atan2((sigma*math.sqrt((1-math.pow(ctheta3,2)))),ctheta3)  # math.atan2 passes y first then x as arguements, may need to switch

            theta2 = math.atan2(z,rad) - math.atan2((L5*math.sin(theta3)),(L4+L5*math.cos(theta3)))

            theta4 = phi-(theta2+theta3)

            if ((-2.3 <= (theta2 - np.pi/2) <= 2.3) and 
        (-2.56 <= theta3 <= 2.56) and
        (-1.76 <= theta4 <= 1.76)):
                joint_angles = [theta1, (theta2 - np.pi/2), theta3, -theta4]
            else:
                joint_angles = [0,0,0,0]

    else:
        joint_angles = [0,0,0,0]

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
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        joint_angles[3]
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

    rate = rospy.Rate(10)
    rospy.spin()


if __name__ == '__main__':
    main()
