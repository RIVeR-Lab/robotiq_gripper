# ROS node wrapper for Airpick gripper
#! /usr/bin/env python

import rospy
from robotiq_gripper.airpick_tcp_control import Register, Gripper


if __name__=="__main__":
    rospy.init_node("airpick_gripper")
    
    # specify the UR robot address here
    address = "192.168.0.2"
    gripper = Gripper(address)




