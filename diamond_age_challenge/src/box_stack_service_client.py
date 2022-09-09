#!/usr/bin/env python

from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from diamond_age_challenge.srv import StackBoxes

# starts node and call for 'stack_boxes' service by sending in names of the cube spawned in request message
def stack_boxes_client():
    rospy.init_node("diamond_age_stacking_client", anonymous=True)
    rospy.wait_for_service('stack_boxes')
    try:
        req_handle=rospy.ServiceProxy('stack_boxes',StackBoxes)
        response=req_handle("first_box","second_box")
        return "Stacking service called"
    except rospy.ServiceException as err:
        print("Service call failed" %err)
    



if __name__=="__main__":
    print("Requesting box to be tacked")
    print(stack_boxes_client())