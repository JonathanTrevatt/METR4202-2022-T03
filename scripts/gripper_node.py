#!/usr/bin/env python3
"""
This script publishes a desired pose to the metr4202 prac wk7 node.
"""

import rospy

from std_msgs.msg import Bool, Float32, Int16
import pigpio

def grip_controller(bool: Bool):
    if bool.data is True:
        rpi.set_servo_pulsewidth(18,2000) 
    else:
        rpi.set_servo_pulsewidth(18,1400) 

def main():
    global rpi
    # Initialise node with any node name
    rospy.init_node('gripper_node') 

    rpi = pigpio.pi()
    rpi.set_mode(18, pigpio.OUTPUT) 
    # rpi.set_servo_pulsewidth(18,1000)
    #1000 is the closed position
    #1500 is the grip box position
    #2000 is the open position

    # Create publisher
    gripperSub = rospy.Subscriber(
        'desired_gripper_state', # Topic name
        Bool, # Message type
        grip_controller # Callback function (required)
    )

    rate = rospy.Rate(10)

    rospy.spin()

if __name__ == '__main__':
    main()

