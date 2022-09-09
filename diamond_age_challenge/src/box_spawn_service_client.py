#!/usr/bin/env python

from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from diamond_age_challenge.srv import SpawnBoxes

# Starts a node and call for 'spawn_boxes' service
def spawn_boxes_client():
    rospy.init_node("diamond_age_service_client", anonymous=True)
    rospy.wait_for_service('spawn_boxes')
    try:
        req_handle=rospy.ServiceProxy('spawn_boxes',SpawnBoxes)
        response=req_handle("Spawn",2)
        return "Success"
    except rospy.ServiceException as err:
        print("Service call failed" %err)
    



if __name__=="__main__":
    print("Requesting box to be spawned")
    print(spawn_boxes_client())
    print("Call stacking service now")