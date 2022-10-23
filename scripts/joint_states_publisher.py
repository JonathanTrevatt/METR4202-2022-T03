#!/usr/bin/env python3
"""
This script publishes a set of random joint states to the dynamixel controller.
Use this to get an idea of how to code your inverse kinematics!
"""

# Always need this
import rospy

# Import message types
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

import numpy as np
import math

"""
inverse_kinematics: A callback function for desired pose subscriber 'sub' that calls the 
inv_kin function to assemble and publish out desired joint angles
Params: pose -> from callback 
"""
def inverse_kinematics(pose: Pose) -> JointState:
    global pub
    global psb
    # TODO: Have fun :)
    psb = np.array([pose.position.x,pose.position.y,pose.position.z])

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

    joint_angles = inv_kin(psb)
    msg = joint_message_assembler(joint_angles)
    pub.publish(msg)

    rospy.loginfo(f'Got desired pose\n[\n\tpos:\n{pose.position}\nrot:\n{pose.orientation}\n]')

"""
inv_kin: Calculates the inverse kinematics of 4R gripper robot 
Params: psb -> desired position array of type (x,y,z)
Return: joint angle solution of each joint in type (angle1, angle2, angle3, angle4)
"""
def inv_kin(psb):
    #joint link lengths from CAD drawings
    L4 = 0.1175
    L5 = 0.095
    L6 = 0.09761
    L7 = 0.015

    # Desired Coordinates
    xd = psb[0]
    yd = psb[1]
    zd = psb[2]

    #list of end-effector configurations to pass to inverse kinematics
    #inverse kinematics solutions with each of these configurations will be tested for validity
    test_phi_1 = 0
    test_phi_2 = -np.pi/8
    test_phi_3 = -np.pi/4
    test_phi_4 = -3*np.pi/8
    test_phi_5 = -np.pi/2

    #array of end-effector configurations --> phi
    phi = [test_phi_5, test_phi_4, test_phi_3, test_phi_2, test_phi_1]

    #init joint angle solution array
    joint_angles_array = []

    #test phi angles and build inverse kinematics solutions array
    for i in range(0,len(phi)):
        #desired radius = sqrt(xd^2 + yd^2)
        rad_des = math.sqrt((math.pow(xd,2)+math.pow(yd,2)))
        #desired height = z_des
        z_des = zd

        # Offset desired radius depending on phi angle
        # This is because the gripper is not in line with the link attached to joint 4
        if (phi[i] == test_phi_5):
            rad_des += L7
        if (phi[i] == test_phi_4):
            rad_des += 0.9*L7
        if (phi[i] == test_phi_3):
            rad_des += 0.8*L7
        if (phi[i] == test_phi_2):
            rad_des += 0.7*L7
        if (phi[i] == test_phi_1):
            z_des += L7

        #Make 2R offset changes to shift desired position of the end effector to the desired position of joint 4
        rad = rad_des - L6*math.cos(phi[i])
        #height offset is quadratic function using the radius which accounts for link sagging
        z_offset = 0.6*rad**2
        #accounts for the distance from base to joint 2
        joint_2_offset = 0.14
        z = z_des - L6*math.sin(phi[i]) - joint_2_offset + z_offset
        rad_2r = math.sqrt((math.pow(rad,2)+math.pow(z,2)))

        #checks if the robot is in its workspace
        if ((L4+L5) >= rad_2r):
            theta1 = math.atan2(yd,xd)

            #Test elbow up solution first
            sigma = -1

            ctheta3 = (math.pow(rad,2)+math.pow(z,2)-math.pow(L4,2)-math.pow(L5,2))/(2*L4*L5)

            # another check on workspace -- atan2 error
            if ((1-math.pow(ctheta3,2)) < 0):
                joint_angles = [10,10,10,10]
            else:
                theta3 = math.atan2((sigma*math.sqrt((1-math.pow(ctheta3,2)))),ctheta3)  # math.atan2 passes y first then x as arguements, may need to switch

                theta2 = math.atan2(z,rad) - math.atan2((L5*math.sin(theta3)),(L4+L5*math.cos(theta3)))

                theta4 = phi[i]-(theta2+theta3)

                #if solution is inside joint angle limits
                if ((-2.3 <= (theta2 - np.pi/2) <= 2.3) and 
                (-2.56 <= theta3 <= 2.56) and
                (-1.76 <= theta4 <= 1.76) and z_des >= 0):
                    #To account for motor mounting in physical robot -- theta2 needs pi/2 subtracted and sign of theta4 needs to be flipped
                    joint_angles = [theta1, (theta2 - np.pi/2), theta3, -theta4]
                else:
                    #Test elbow down solution
                    sigma = 1

                    ctheta3 = (math.pow(rad,2)+math.pow(z,2)-math.pow(L4,2)-math.pow(L5,2))/(2*L4*L5)

                    theta3 = math.atan2((sigma*math.sqrt((1-math.pow(ctheta3,2)))),ctheta3)  # math.atan2 passes y first then x as arguements, may need to switch

                    theta2 = math.atan2(z,rad) - math.atan2((L5*math.sin(theta3)),(L4+L5*math.cos(theta3)))

                    theta4 = phi[i]-(theta2+theta3)

                    if ((-2.3 <= (theta2 - np.pi/2) <= 2.3) and 
                (-2.56 <= theta3 <= 2.56) and
                (-1.76 <= theta4 <= 1.76) and z_des >= 0):
                        joint_angles = [theta1, (theta2 - np.pi/2), theta3, -theta4]
                    else:
                        #if outside joint limits, make dynamixel ignore the joint angles
                        joint_angles = [10,10,10,10]

        else:
            joint_angles = [10,10,10,10]

        #construct joint angles solution array
        joint_angles_array.append(joint_angles)

    joint_angles_solution = []

    #pick the first valid joint angle solution
    for j in range(0,len(joint_angles_array)):
        if not (joint_angles_array[j] == [10,10,10,10]):
            joint_angles_solution = joint_angles_array[j]
            break
        else:
            joint_angles_solution = [10,10,10,10]

    return joint_angles_solution

"""
Joint message assembler: assembles joint angle message from inv_kin joint angle solution
Params: joint_angles -> joint angle solution returned from inv_kin
Return: JointState message
"""
def joint_message_assembler(joint_angles) -> JointState:
    # Create message of type JointState
    msg = JointState(
        # Set header with current time
        header=Header(stamp=rospy.Time.now()),
        # Specify joint names (see `controller_config.yaml` under `dynamixel_interface/config`)
        name=['joint_1', 'joint_2', 'joint_3', 'joint_4']
    )
    #assign angles to message
    msg.position = [
        joint_angles[0],
        joint_angles[1],
        joint_angles[2],
        joint_angles[3]
    ]
    return msg


def main():
    global pub
    # Initialise node
    rospy.init_node('joint_node')
    # Create desired joint states publisher
    pub = rospy.Publisher(
        'desired_joint_states', # Topic name
        JointState, # Message type
        queue_size=10 # Topic size (optional)
    )

    # Create subscriber listening to desired pose (see desired_pose_node)
    sub = rospy.Subscriber(
        'desired_pose', # Topic name
        Pose, # Message type
        inverse_kinematics # Callback function (required)
    )
    #set rate to 50 Hz, this is common to all nodes
    rate = rospy.Rate(50)
    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()


if __name__ == '__main__':
    main()