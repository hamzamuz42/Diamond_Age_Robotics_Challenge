#!/usr/bin/env python
from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from diamond_age_challenge.srv import StackBoxes, StackBoxesResponse


class StackingClass(object):
    def __init__(self) -> None:
        # Defining settings to communicate with MoveIt commander
        rospy.init_node("stacking_node", anonymous=True)
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

        # Subscribing to topics with spawned box's pose
        self.first_cube_pose_subs = rospy.Subscriber('/first_box', 
                    geometry_msgs.msg.PoseStamped, self.first_cube_pose_callback, queue_size = 10)
        self.second_cube_pose_subs = rospy.Subscriber('/second_box', 
                    geometry_msgs.msg.PoseStamped, self.second_cube_pose_callback, queue_size = 10)
        
        # Defining service and callback function
        rospy.Service('stack_boxes',StackBoxes, self.stack_server)

    # Continuously updates the cube pose messages
    def first_cube_pose_callback(self,msg):
        self.first_cube_pose.position.x=(msg.pose.position.x)
        self.first_cube_pose.position.y=(msg.pose.position.y)
        self.first_cube_pose.position.z=(msg.pose.position.z+0.12)
    def second_cube_pose_callback(self,msg):
        self.second_cube_pose.position.x=(msg.pose.position.x)
        self.second_cube_pose.position.y=(msg.pose.position.y)
        self.second_cube_pose.position.z=(msg.pose.position.z+0.18)

    # Sends robot to initial pose for next call of action/service
    def initial_pose(self):
        self.move_group.set_named_target("ready")
        result = self.move_group.go(wait=True)
        return result
        
    # Opens robot gripper
    def open_gripper(self):
        self.hand_group.set_named_target("open")
        result = self.hand_group.go(wait=True)
        return result
    
    # Moves the robot gripper to first cube pose and grips it 
    def move_to_cube(self,cube_names):
        self.move_group.set_pose_target(self.first_cube_pose)
        result = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        touch_links_hand = self.robot.get_link_names(group="panda_hand")
        self.scene.attach_box(self.eef_link, cube_names.first_box_name, touch_links=touch_links_hand)
        return result
    
    # moves robot EE to second cube position and stacks the first cube
    def move_to_second_cube(self,cube_names):
        self.move_group.set_pose_target(self.second_cube_pose)
        result = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        self.scene.remove_attached_object(self.eef_link, cube_names.first_box_name)
        return result

    def close_gripper(self):
        self.hand_group.set_named_target("close")
        result = self.hand_group.go(wait=True)
        return result

    # Service callback function
    def stack_server(self,req):
        open_gripper=self.open_gripper()
        move_to_cube=self.move_to_cube(req)
        close_gripper=self.close_gripper()
        stack=self.move_to_second_cube(req)
        open_gripper=self.open_gripper()
        move_initial=self.initial_pose()
        return "Boxes stacked"



def main():
    server=StackingClass()
    while not rospy.is_shutdown():
       rospy.spin()

if __name__=="__main__":
    main()

