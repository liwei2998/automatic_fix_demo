#!/usr/bin/env python

import sys
import get_ik
import copy
import moveit_commander
import rospy
import moveit_msgs.msg
import time
import tf
import numpy as np
import geometry_msgs.msg
import pybullet as p
import trajectory_msgs.msg
from moveit_msgs.msg import PlanningScene, PlanningSceneComponents, ObjectColor, Constraints, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String,Empty,UInt16
import random
from math import pi
import matplotlib.pyplot as plt
import test_config as tc
import test_planner as tp
import help_func as hf

def scale_trajectory_speed(traj, scale):
    new_traj = moveit_msgs.msg.RobotTrajectory()
    new_traj.joint_trajectory = traj.joint_trajectory
    n_joints = len(traj.joint_trajectory.joint_names)
    n_points = len(traj.joint_trajectory.points)
    points = list(traj.joint_trajectory.points)

    for i in range(n_points):
        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.time_from_start = traj.joint_trajectory.points[i].time_from_start / scale
        point.velocities = list(traj.joint_trajectory.points[i].velocities)
        point.accelerations = list(traj.joint_trajectory.points[i].accelerations)
        point.positions = traj.joint_trajectory.points[i].positions

        for j in range(n_joints):
            point.velocities[j] = point.velocities[j] * scale
            point.accelerations[j] = point.accelerations[j] * scale * scale
        points[i] = point

    new_traj.joint_trajectory.points = points
    return new_traj

def robkong_go_to_home(times,vel):
    if times == 1:
        joint_values = [-1.24, -2.23, -1.6,-1.13,1.46,1.13]
    elif times == 2:
        joint_values = [-2.03, -1.1, 1.93, -2.42, -1.5, 1.23]
    group2.set_joint_value_target(joint_values)
    plan = group2.plan()
    new_traj = scale_trajectory_speed(plan, vel)
    group2.execute(new_traj)
    group2.stop()
    group2.clear_pose_targets()

def robhong_go_to_home(times,vel):
    if times == 1:
        joint_values = [0.96, -1.07, 1.37, -1.31, -1.7, 0.32]
    elif times == 2:
        joint_values = [-2.04,-1.28,2.33,-2.69,-1.56,0.84]
    group1.set_joint_value_target(joint_values)
    plan = group1.plan()
    new_traj = scale_trajectory_speed(plan, vel)
    group1.execute(new_traj)
    group1.stop()
    group1.clear_pose_targets()

def hybridPub(joint_value,robot_arm):
    if robot_arm == "robhong":
        group = group3
    elif robot_arm == "robkong":
        group = group4
    else:
        print "robot_arm input error, please input valid robot_arm name"
        return 0
    group.set_joint_value_target([joint_value])
    group.go(wait=True)

def addCollisionObjects():
    global paper_pose
    global paper_name
    paper_pose = geometry_msgs.msg.PoseStamped()
    paper_pose.header.frame_id = "world"
    paper_pose.pose.orientation.w = 1.0
    paper_pose.pose.position.x = 0
    paper_pose.pose.position.y = 0
    paper_pose.pose.position.z = 0.706
    paper_name = 'paper'
    scene.add_box(paper_name, paper_pose, (0.21, 0.297, 0.002))

#################### INITIALIZATION #######################
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('dual_arm_origami', anonymous=True)
listener = tf.TransformListener()
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
rospy.sleep(2)
group_name1 = "hong_arm"
group1 = moveit_commander.MoveGroupCommander(group_name1)
group_name2 = "kong_arm"
group2 = moveit_commander.MoveGroupCommander(group_name2)
group_name3 = "hong_hand"
group3 = moveit_commander.MoveGroupCommander(group_name3)
group_name4 = "kong_hand"
group4 = moveit_commander.MoveGroupCommander(group_name4)
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
rospy.sleep(1)
addCollisionObjects()
rospy.sleep(1)
robkong_go_to_home(1,0.4)
robhong_go_to_home(1,0.4)
rospy.sleep(1)
hybridPub(0.6981,"robhong")
hybridPub(0.6981,"robkong")
# scene.remove_world_object(paper_name)
#################### INITIALIZATION #######################

################### find the max distance config ##################
center = group2.get_current_pose().pose
robot_state = robot.get_current_state()
poly = ((0.105,0.145,0.71),(0.105,-0.145,0.71),(-0.105,-0.145,0.71),(-0.105,0.145,0.71))
configs = hf.config_generate(group_name1,center,robot_state,poly,num=35)
p.disconnect()
k_start = [-1.46, -2.36, -1.52, -0.82, 1.81, 2.65, 0.7]
# k_start = [-1.75, -1.95, -1.34, -1.4, 1.7, 0.22, 0.7]
h_start = [0.96, -1.07, 1.37, -1.31, -1.7, 0.32, 0.7]
dis = hf.get_dis(configs,k_start,h_start,p.DIRECT)
p.disconnect()
index = dis.index(max(dis))
sid = sorted(range(len(dis)),key=lambda k:dis[k], reverse=True)
goal_conf = configs[index][:6]
print goal_conf
################### find the max distance config ##################

################### simulation ##################
print "-------------simulation--------------"
print "please see the arm's movement in the Moveit"
group2.set_joint_value_target(k_start[:6])
group2.go()
rospy.sleep(0.5)
print "the joint values of arm hong is ", goal_conf
group1.set_joint_value_target(goal_conf)
group1.go()
################### simulation ##################
