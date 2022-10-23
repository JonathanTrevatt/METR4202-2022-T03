#!/usr/bin/env python3
"""
This script sets the PWM to control the servo on the Raspberry Pi.
"""

# Imports

import rospy
from std_msgs.msg import Bool
import pigpio

"""
grip_controller: callback function which toggles gripper 'open' or 'closed' based on boolean flag

Params: bool -> 'True' (open) or 'False' (closed)
"""
def grip_controller(bool: Bool):
    #1200 is the grip box position
    #2000 is the open position
    if bool.data is True:
        rpi.set_servo_pulsewidth(18,2000) 
    else:
        rpi.set_servo_pulsewidth(18,1200) 

def main():
    global rpi
    # Initialise node
    rospy.init_node('gripper_node') 

    #initialise raspberry pi GPIO
    rpi = pigpio.pi()
    rpi.set_mode(18, pigpio.OUTPUT) 

    # Create subscriber listening to desired gripper state
    gripperSub = rospy.Subscriber(
        'desired_gripper_state', # Topic name
        Bool, # Message type
        grip_controller # Callback function (required)
    )
    #set rate to 50 Hz, this is common to all nodes
    rate = rospy.Rate(50)
    # You spin me right round baby, right round...
    # Just stops Python from exiting and executes callbacks
    rospy.spin()

if __name__ == '__main__':
    main()

