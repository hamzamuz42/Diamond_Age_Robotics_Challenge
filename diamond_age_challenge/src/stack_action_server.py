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

from diamond_age_challenge.msg import BoxStackerServerAction, BoxStackerServerFeedback, BoxStackerServerResult
class StackActionServer(object):
    def __init__(self) -> None:
        rospy.init_node("diamond_age_stack_action", anonymous=True)

        # Robot settings to communicate with MoveIt commander
        self.robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        self.scene = scene
        self.move_group = move_group
        self.planning_frame = move_group.get_planning_frame()
        self.eef_link = move_group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()
        self.hand_group = moveit_commander.MoveGroupCommander("panda_hand")   
        self.pose_goal = geometry_msgs.msg.Pose()
        self.first_cube_pose=self.move_group.get_current_pose().pose
        self.second_cube_pose=self.move_group.get_current_pose().pose
        self.first_cube_height=0.12
        self.second_cube_height=0.18

        # Subscribing to topics with spawned box's pose
        self.first_cube_pose_subs = rospy.Subscriber('/first_box', 
                    geometry_msgs.msg.PoseStamped, self.first_cube_pose_callback, queue_size = 10)
        self.second_cube_pose_subs = rospy.Subscriber('/second_box', 
                    geometry_msgs.msg.PoseStamped, self.second_cube_pose_callback, queue_size = 10)

        # Defining action and callback function
        self.act_server=actionlib.SimpleActionServer("box_stack_act",BoxStackerServerAction,
                                                        execute_cb=self.stack_server, auto_start=False)
        self.act_server.start()
        print("action should start")

    # Continuously updates the first cube pose message
    def first_cube_pose_callback(self,msg):
        self.first_cube_pose.position.x=(msg.pose.position.x)
        self.first_cube_pose.position.y=(msg.pose.position.y)
        self.first_cube_pose.position.z=(msg.pose.position.z+self.first_cube_height)
    # Continuously updates the second cube pose message    
    def second_cube_pose_callback(self,msg):
        self.second_cube_pose.position.x=(msg.pose.position.x)
        self.second_cube_pose.position.y=(msg.pose.position.y)
        self.second_cube_pose.position.z=(msg.pose.position.z+self.second_cube_height)

    # Sends robot to initial pose for next call of action/service
    def initial_pose(self):
        self.move_group.set_named_target("ready")
        result = self.move_group.go(wait=True)
        return "Back to initial pose"
    
    # Opens robot gripper
    def open_gripper(self):
        self.hand_group.set_named_target("open")
        result = self.hand_group.go(wait=True)
        return "Gripper opened"

    # Moves the robot gripper to first cube pose and grips it    
    def move_to_cube(self):
        self.move_group.set_pose_target(self.first_cube_pose)
        result = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        touch_links_hand = self.robot.get_link_names(group="panda_hand")
        self.scene.attach_box(self.eef_link, "first_box", touch_links=touch_links_hand)
        return "First cube approached"
    
    # moves robot EE to second cube position and stacks the first cube
    def move_to_second_cube(self):
        self.move_group.set_pose_target(self.second_cube_pose)
        result = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        self.scene.remove_attached_object(self.eef_link, "first_box")
        return "Moved to second cube"

    def close_gripper(self):
        self.hand_group.set_named_target("close")
        result = self.hand_group.go(wait=True)
        return "Gripper closed"

    # Action callback function
    def stack_server(self,req):
        complete=True
        feedback=BoxStackerServerFeedback()
        result=BoxStackerServerResult()
        if self.act_server.is_preempt_requested():
            self.act_server.set_preempted()
            complete=False
        
        feedback.state=self.open_gripper()
        self.act_server.publish_feedback(feedback)
        feedback.state=self.move_to_cube()
        self.act_server.publish_feedback(feedback)
        feedback.state=self.close_gripper()
        self.act_server.publish_feedback(feedback)
        feedback.state=self.move_to_second_cube()
        self.act_server.publish_feedback(feedback)
        feedback.state=self.open_gripper()
        self.act_server.publish_feedback(feedback)
        feedback.state=self.initial_pose()
        self.act_server.publish_feedback(feedback)
        if complete:
            result.success="Spawned Call stacking action now"
            self.act_server.set_succeeded(result)   


            



def main():
    action_server=StackActionServer()    
    while not rospy.is_shutdown():
       rospy.spin()

if __name__=="__main__":
    main()