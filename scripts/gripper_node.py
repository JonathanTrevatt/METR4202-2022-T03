#!/usr/bin/env python3
"""
This script publishes a desired pose to the metr4202 prac wk7 node.
"""

import rospy

from std_msgs.msg import Bool, Float32
import pigpio

def grip_controller(bool: Bool) -> Float32:
    global gripperPub
    gripOpen = Float32()
    gripOpen.data = 2000

    gripClose = Float32()
    gripClose.data = 1000

    if bool.data is True:
        rpi.set_servo_pulsewidth(18,1500) 
    else:
        rpi.set_servo_pulsewidth(18,1000) 

def main():
    global gripperPub
    global rpi 
    rpi = pigpio.pi()
    rpi.set_mode(18, pigpio.OUTPUT) 
    # rpi.set_servo_pulsewidth(18,1000)
    #1000 is the closed position
    #1500 is the grip box position
    #2000 is the open position

    # Initialise node with any node name
    rospy.init_node('gripper_node')
    # Create publisher
    gripperSub = rospy.Subscriber(
        'desired_gripper_state', # Topic name
        Bool, # Message type
        grip_controller # Callback function (required)
    )
    gripperPub = rospy.Publisher("gripper", Float32, queue_size=10)
    rate = rospy.Rate(10)

    rospy.spin()

if __name__ == '__main__':
    main()

