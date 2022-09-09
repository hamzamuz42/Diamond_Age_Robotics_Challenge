#!/usr/bin/env python

from __future__ import print_function

import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import String
from random import seed, random

from diamond_age_challenge.srv import SpawnBoxes, SpawnBoxesResponse


class SpawnClass(object):
    def __init__(self) -> None:
        rospy.init_node("diamond_age_service")
        self.scene = moveit_commander.PlanningSceneInterface()

        # Defining reachable workspace for the cubes to spawn in
        self.max_ws_bound=0.38
        self.min_ws_bound=-0.38
        self.inner_ws_bound=0.2
        self.first_box=rospy.Publisher('/first_box',geometry_msgs.msg.PoseStamped,queue_size=1,latch=True)
        self.second_box=rospy.Publisher('/second_box',geometry_msgs.msg.PoseStamped,queue_size=5,latch=True)

    # generate random coordinates within reachable workspace of the robot
    def spawn_location_generate(self):
        seed(random())
        pose=round(self.min_ws_bound + (random() * (self.max_ws_bound - self.min_ws_bound)),2)
        return pose+self.inner_ws_bound if pose>=0 else pose-self.inner_ws_bound

    # Spawn 2 boxes at random pose in the planning scene
    def spawn_boxes(self, req):
        scene = self.scene
        box_pose_1 = geometry_msgs.msg.PoseStamped()
        box_pose_1.header.frame_id = "world"
        box_pose_1.pose.orientation.w = 1.0
        
        box_pose_2 = geometry_msgs.msg.PoseStamped()
        box_pose_2.header.frame_id = "world"
        box_pose_2.pose.orientation.w = 1.0
            
        box_pose_1.pose.position.x=self.spawn_location_generate()
        box_pose_1.pose.position.y=self.spawn_location_generate()
        box_pose_1.pose.position.z = 0.0375  # above the world frame
        
        box_name = "first_box"
        scene.add_box(box_name, box_pose_1, size=(0.06, 0.06, 0.06))
        self.first_box.publish(box_pose_1)
        print("first box: ",box_pose_1)
        
        box_name = "second_box"
        box_pose_2.pose.position.x=self.spawn_location_generate()
        box_pose_2.pose.position.y=self.spawn_location_generate()
        box_pose_2.pose.position.z = 0.0375  # above the world frame
        scene.add_box(box_name, box_pose_2, size=(0.06, 0.06, 0.06))
        self.second_box.publish(box_pose_2)
        print("second box: ",box_pose_2)
        
        return "success"

    # Defines service and callback function
    def spawn_boxes_server(self):
        rospy.Service('spawn_boxes',SpawnBoxes, self.spawn_boxes)
        print("Box spawner service ready")
        return True

def main():
    server=SpawnClass()
    spawn=server.spawn_boxes_server()
    
    while not rospy.is_shutdown():
       rospy.spin()

if __name__=="__main__":
    main()