#! /usr/bin/env python
""" ROS node wrapper for Airpick gripper """
import rospy
from robotiq_gripper.airpick_tcp_control import Gripper
from robotiq_gripper.srv import AirpickSimpleControl, AirpickSimpleControlResponse 

class AirpickNode(object):
    def __init__(self):
        address = "192.168.0.2"
        self.gripper = Gripper(address)
        
        #ROS services
        self.control_server = rospy.Service('/airpick_control_service', AirpickSimpleControl, self.__airpick_control_server)
        rospy.loginfo("Airpick control server started")

    def __airpick_control_server(self, req):
        resp = False
        if req.command == "grip":
            resp = self.gripper.vacuum_grip()
        elif req.command == "release":
            resp = self.gripper.vacuum_release()

        return AirpickSimpleControlResponse(resp)

if __name__=="__main__":
    rospy.init_node("airpick_gripper")
    ap = AirpickNode()

    rospy.spin()




