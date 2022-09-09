#!/usr/bin/env python

from __future__ import print_function

import sys
import copy
import rospy
import actionlib
import moveit_commander
import geometry_msgs.msg
from std_msgs.msg import String
from random import seed, random

from diamond_age_challenge.msg import BoxSpawnerServerAction, BoxSpawnerServerFeedback, BoxSpawnerServerResult
class ActionServer():
    def __init__(self) -> None:
        rospy.init_node("diamond_age_spawn_action", anonymous=True)
        self.scene = moveit_commander.PlanningSceneInterface()
        
        # Defining reachable workspace for the cubes to spawn in
        self.max_ws_bound=0.38
        self.min_ws_bound=-0.38
        self.inner_ws_bound=0.2
        self.first_box=rospy.Publisher('/first_box',geometry_msgs.msg.PoseStamped,queue_size=1,latch=True)
        self.second_box=rospy.Publisher('/second_box',geometry_msgs.msg.PoseStamped,queue_size=5,latch=True)

        # defining action and callback function
        self.act_server=actionlib.SimpleActionServer("box_spawn_act",BoxSpawnerServerAction,
                                                        execute_cb=self.box_spawn_act, auto_start=False)
        self.act_server.start()

    # Action callback function
    def box_spawn_act(self,req):
        complete=True
        feedback=BoxSpawnerServerFeedback()
        result=BoxSpawnerServerResult()

        # If action is called pree,ptively raise error 
        if self.act_server.is_preempt_requested():
            self.act_server.set_preempted()
            complete=False
        
        feedback.state=self.spawn_boxes()
        self.act_server.publish_feedback(feedback)
        if complete:
            result.success="Spawned Call stacking action now"
            self.act_server.set_succeeded(result) 

    # generate random coordinates within reachable workspace of the robot
    def spawn_location_generate(self):
        seed(random())
        pose=round(self.min_ws_bound + (random() * (self.max_ws_bound - self.min_ws_bound)),2)
        return pose+self.inner_ws_bound if pose>=0 else pose-self.inner_ws_bound

    # Spawn 2 boxes at random pose in the planning scene
    def spawn_boxes(self):
        scene = self.scene
        box_pose_1 = geometry_msgs.msg.PoseStamped()
        box_pose_1.header.frame_id = "world"
        box_pose_1.pose.orientation.w = 1.0
        
        box_pose_2 = geometry_msgs.msg.PoseStamped()
        box_pose_2.header.frame_id = "world"
        box_pose_2.pose.orientation.w = 1.0
            
        box_pose_1.pose.position.x=self.spawn_location_generate()
        box_pose_1.pose.position.y=self.spawn_location_generate()
        box_pose_1.pose.position.z = 0.0375  # above the world frame so box is on surface
        
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
        
        return "Boxes spawned"

def main():
    action_server=ActionServer()    
    while not rospy.is_shutdown():
       rospy.spin()

if __name__=="__main__":
    main()