#!/usr/bin/env python
import sys
import rospy
import actionlib
from std_msgs.msg import Int8
# from ur_moveit_myplan.msg import qr_status
# from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import sqrt, pi, acos, sin, cos
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped, Pose
from math import sqrt, pi, acos, sin, cos, atan2, tan
from std_msgs.msg import String,Empty,UInt16
import numpy as np
import message_filters
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
from arc_rotate import *
# from apriltag_ros.msg import AprilTagDetection, AprilTagDetectionArray
from std_msgs.msg import *
import random
from math import pi
import matplotlib.pyplot as plt
from shapely.geometry import *
import help_func as hf
import get_ik
import pybullet as p
import roslib; roslib.load_manifest('ur_driver')
roslib.load_manifest('robotic_origami')
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('dual_arm_origami', anonymous=True)
rospy.sleep(2)
import tf
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
###set rigid gripper pose group.set_joint_value_target([1]) ---> group.go()
display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

global odom_c_pub
# diaplay center point
odom_c_pub = rospy.Publisher("/odom_c", Odometry, queue_size=50)

#global motion group
global display_trajectory_trigger_pub
display_trajectory_trigger_pub = rospy.Publisher(
                                      '/display_trigger',
                                      String,
                                      queue_size=20)

print "============  Start now"

def gripper_pose(angle,gripper_name):
    ##input angle in degree
    if gripper_name == "hong_hand":
        group = group3
    elif gripper_name == "kong_hand":
        group = group4
    else:
        print "robot_arm input error, please input valid robot_arm name"
    angle=angle*pi/180
    group.set_joint_value_target([angle])
    result=group.go()
    print "============ Gripper: %s target: %s " % (gripper_name, angle)
    print "============ result:%s" % result

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

def move_target(x, y, z, ox, oy, oz, ow, vel,robot_arm):
    if robot_arm == "robhong":
        group = group1
    elif robot_arm == "robkong":
        group = group2
    else:
        print "robot_arm input error, please input valid robot_arm name"
    return 0
    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = ox
    pose_target.orientation.y = oy
    pose_target.orientation.z = oz
    pose_target.orientation.w = ow
    pose_target.position.x = x
    pose_target.position.y = y
    pose_target.position.z = z
    group.set_pose_target(pose_target)
    plan = group.plan()
    scaled_traj = scale_trajectory_speed(plan, vel)
    print "move_target"
    group.execute(scaled_traj)

def move_waypoints(dx,dy,dz,vel,robot_arm):
    if robot_arm == "robhong":
        group = group1
    elif robot_arm == "robkong":
        group = group2
    else:
        print "robot_arm input error, please input valid robot_arm name"
        return 0
    waypoints = []
    waypoints.append(group.get_current_pose().pose)
    wpose = copy.deepcopy(group.get_current_pose().pose)
    wpose.position.x += dx
    wpose.position.y += dy
    wpose.position.z += dz
    waypoints.append(copy.deepcopy(wpose))
    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.02, 0.0)
    new_traj = scale_trajectory_speed(plan,vel)
    result=group.execute(new_traj)
    print "move waypoint result"
    print dx,dy,dz,robot_arm
    print result

def robhong_go_to_home(state):
    if state == 1:
        joint_values = [-66.86/180*pi, -69.6/180*pi, -120.2/180*pi, -51.78/180*pi, 77.87/180*pi, 111.00/180*pi]
    elif state == 2:
        joint_values = []
    group1.set_joint_value_target(joint_values)
    plan = group1.plan()
    print "============ hong_home"
    rospy.sleep(2)
    scaled_traj2 = scale_trajectory_speed(plan, 1)
    group1.execute(scaled_traj2)

def robkong_go_to_home(state):
    if state == 1:
        joint_values = [45.80/180*pi, -108.10/180*pi, 111.35/180*pi, -113.3/180*pi, -72.05/180*pi, 47.2/180*pi]
    elif state == 2:
        joint_values = []
    group2.set_joint_value_target(joint_values)
    plan = group2.plan()
    print "============ kong_home"
    rospy.sleep(2)
    scaled_traj2 = scale_trajectory_speed(plan, 1)
    group2.execute(scaled_traj2)

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

def group_rotate_by_external_axis(center_point, axis, total_angle,robot_arm):
    global odom_c_pub
    if robot_arm == "robhong":
        group = group1
    elif robot_arm == "robkong":
        group = group2
    else:
        print "robot_arm input error, please input valid robot_arm name"
        return 0
    pose_target = group.get_current_pose().pose

    waypoints_new = calc_waypoints_ARC(pose_target, center_point, axis, total_angle, odom_c_pub)
    # Before the execution of the real robot, turn on the display of the end effector's position and orientation
    # display end effector's trajectory
    # subcriber of display_trajectory_trigger and corresponding function is implemented in 'display_markers.py'
    display_trajectory_trigger_pub.publish('on')
    rospy.sleep(1)

    # Utilize 'compute_cartesian_path' to get a smooth trajectory
    (plan3, fraction) = group.compute_cartesian_path(
                           waypoints_new,   # waypoints to follow
                           0.01,        # eef_step
                           0.0)         # jump_threshold

    # Move the robot with the ARC trajectory
    caled_plan = scale_trajectory_speed(plan3, 0.3)
    group.execute(caled_plan)
    rospy.sleep(1)

    # Stop displaying end effector's posision and orientation
    display_trajectory_trigger_pub.publish('close')

def move_along_axis(axis,dist,robot_arm):
    if robot_arm == "robhong":
        group = group1
    elif robot_arm == "robkong":
        group = group2
    else:
        print "robot_arm input error, please input valid robot_arm name"
    targ_x=axis[0]*dist
    targ_y=axis[1]*dist
    targ_z=axis[2]*dist
    move_waypoints(targ_x,targ_y,targ_z,0.35,robot_arm)

def showpoint(ppose,pname,ver=0):
    if ver == 1:
        size = (0.01,0.01,0.05)
    else:
        size = (0.01, 0.01, 0.01)
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = "world"
    pose.pose.orientation.w = 1.0
    pose.pose.position.x = ppose[0]
    pose.pose.position.y = ppose[1]
    pose.pose.position.z = ppose[2]
    name = pname
    scene.add_box(name,pose,size)

def descend_to_desktop(robot_arm,gripper_state,margin):
    if robot_arm == "robhong":
        group = group1
        pinch_tip='pinch_tip_hong'
    elif robot_arm == "robkong":
        group = group2
        pinch_tip='pinch_tip_kong'
    else:
        print "robot_arm input error, please input valid robot_arm name"

    current_z=group.get_current_pose().pose.position.z

    if gripper_state== "normal":
        target_z = 0.840+margin
        dz=target_z-current_z
    elif gripper_state=="pinch":
        target_z =0.704+margin
        listener.waitForTransform("world", pinch_tip, rospy.Time(), rospy.Duration(4.0))
        (trans_pinch,rot_) = listener.lookupTransform("world", pinch_tip, rospy.Time(0))
        dz=target_z-trans_pinch[2]
    elif gripper_state=="pinch2":
        target_z =0.702+margin
        dz=target_z-current_z
    else:
        print "wrong gripper state"

    move_waypoints(0,0,dz,0.2,robot_arm)
    rospy.sleep(3)
    print "=========== %s descend_to_desktop" % robot_arm

def fold(global_axis,degree,robot_arm):
    trans_soft=[]
    if robot_arm == "robhong":
        group = group1
        listener.waitForTransform("world", "pinch_tip_hong", rospy.Time(), rospy.Duration(4.0))
        (trans_soft,rot_) = listener.lookupTransform("world", "pinch_tip_hong", rospy.Time(0))
    elif robot_arm == "robkong":
        group = group2
        listener.waitForTransform("world", "pinch_tip_kong", rospy.Time(), rospy.Duration(4.0))
        (trans_soft,rot_) = listener.lookupTransform("world", "pinch_tip_kong", rospy.Time(0))
    else:
        print "robot_arm input error, please input valid robot_arm name"
        return 0

    group_rotate_by_external_axis(trans_soft,global_axis,degree,robot_arm)

def fix(k_start,h_start,tag_pos,tag_rot):
    center = group2.get_current_pose().pose
    robot_state = robot.get_current_state()
    #step1: get valid configurations
    configs = hf.config_generate(group_name1,center,k_start,robot_state,tag_pos,tag_rot,num=10,sim=True)
    #disconnect pybullet
    p.disconnect()
    rospy.sleep(0.5)
    #step2: for each config, return the closest distance.
    # return dis is a list that restores closest distance of all input configs
    dis = hf.get_dis(configs,k_start,h_start,p.DIRECT)
    #disconnect pybullet
    p.disconnect()
    # select the config  that has smallest distance
    sid = sorted(range(len(dis)),key=lambda k:dis[k], reverse=True)
    goal_conf = configs[sid[0]][:6]
    # print "-------%s--------", time.time() - start_time
    print "goal_conf",goal_conf
    rospy.sleep(1)
    #step3: go to the fix configuration
    group1.set_joint_value_target(goal_conf[:6])
    plan = group1.plan()
    new_traj = scale_trajectory_speed(plan, 0.2)
    group1.execute(new_traj)
    group1.stop()
    group1.clear_pose_targets()

def infer_ik(group2_pose,perp_axis,crease_perp_l):
    # this is to obtain configurations along kong's trajectory
    ik = get_ik.GetIK(group_name2)
    # 1: pregrasp's ik
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = group2_pose.position.x
    pose.pose.position.y = group2_pose.position.y
    pose.pose.position.z = group2_pose.position.z
    pose.pose.orientation.x = group2_pose.orientation.x
    pose.pose.orientation.y = group2_pose.orientation.y
    pose.pose.orientation.z = group2_pose.orientation.z
    pose.pose.orientation.w = group2_pose.orientation.w
    m = ik.get_ik(pose,robot.get_current_state())
    m = m[7:13]
    m.append(0)
    print "ik1",m
    rospy.sleep(0.5)
    # 2: move 0.01 upwards's ik
    pose1 = PoseStamped()
    pose1.header.frame_id = "world"
    pose1.header.stamp = rospy.Time.now()
    pose1.pose.position.x = group2_pose.position.x
    pose1.pose.position.y = group2_pose.position.y
    pose1.pose.position.z = group2_pose.position.z + 0.01
    pose1.pose.orientation.x = group2_pose.orientation.x
    pose1.pose.orientation.y = group2_pose.orientation.y
    pose1.pose.orientation.z = group2_pose.orientation.z
    pose1.pose.orientation.w = group2_pose.orientation.w
    m1 = ik.get_ik(pose1,robot.get_current_state())
    m1 = m1[7:13]
    m1.append(0)
    print "ik2",m1
    rospy.sleep(0.5)
    # 3: simplified rotate's ik
    pose2 = PoseStamped()
    pose2.header.frame_id = "world"
    pose2.header.stamp = rospy.Time.now()
    pose2.pose.position.x = group2_pose.position.x
    pose2.pose.position.y = group2_pose.position.y
    pose2.pose.position.z = group2_pose.position.z + 0.03
    pose2.pose.orientation.x = group2_pose.orientation.x
    pose2.pose.orientation.y = group2_pose.orientation.y
    pose2.pose.orientation.z = group2_pose.orientation.z
    pose2.pose.orientation.w = group2_pose.orientation.w
    m2 = ik.get_ik(pose2,robot.get_current_state())
    m2 = m2[7:13]
    m2.append(0)
    print "ik3",m2
    rospy.sleep(0.5)
    # 4: simplified rotate's ik
    pose3 = PoseStamped()
    pose3.header.frame_id = "world"
    pose3.header.stamp = rospy.Time.now()
    pose3.pose.position.x = group2_pose.position.x
    pose3.pose.position.y = group2_pose.position.y
    pose3.pose.position.z = group2_pose.position.z + 0.05
    pose3.pose.orientation.x = group2_pose.orientation.x
    pose3.pose.orientation.y = group2_pose.orientation.y
    pose3.pose.orientation.z = group2_pose.orientation.z
    pose3.pose.orientation.w = group2_pose.orientation.w
    m3 = ik.get_ik(pose3,robot.get_current_state())
    m3 = m3[7:13]
    m3.append(0)
    print "ik4",m3
    rospy.sleep(0.5)
    # 5: simplified rotate's ik
    pose5 = PoseStamped()
    pose5.header.frame_id = "world"
    pose5.header.stamp = rospy.Time.now()
    pose5.pose.position.x = group2_pose.position.x
    pose5.pose.position.y = group2_pose.position.y
    pose5.pose.position.z = group2_pose.position.z + 0.095
    pose5.pose.orientation.x = group2_pose.orientation.x
    pose5.pose.orientation.y = group2_pose.orientation.y
    pose5.pose.orientation.z = group2_pose.orientation.z
    pose5.pose.orientation.w = group2_pose.orientation.w
    m5 = ik.get_ik(pose5,robot.get_current_state())
    m5 = m5[7:13]
    m5.append(0)
    print "ik5",m5
    rospy.sleep(0.5)
    # 6: final grasp's ik
    dist_x = -perp_axis[0]*crease_perp_l
    dist_y = -perp_axis[1]*crease_perp_l
    dist_z = -perp_axis[2]*crease_perp_l
    pose4 = PoseStamped()
    pose4.header.frame_id = "world"
    pose4.header.stamp = rospy.Time.now()
    pose4.pose.position.x = pose.pose.position.x + dist_x
    pose4.pose.position.y = pose.pose.position.y + dist_y
    pose4.pose.position.z = pose.pose.position.z + dist_z
    pose4.pose.orientation.x = pose.pose.orientation.x
    pose4.pose.orientation.y = pose.pose.orientation.y
    pose4.pose.orientation.z = pose.pose.orientation.z
    pose4.pose.orientation.w = pose.pose.orientation.w
    m4 = ik.get_ik(pose4,robot.get_current_state())
    m4 = m4[7:13]
    m4.append(0)
    print "ik6",m4
    rospy.sleep(0.5)
    # 7: simplified rotate's ik
    pose7 = PoseStamped()
    pose7.header.frame_id = "world"
    pose7.header.stamp = rospy.Time.now()
    pose7.pose.position.x = pose.pose.position.x+ dist_x
    pose7.pose.position.y = pose.pose.position.y+ dist_y
    pose7.pose.position.z = pose.pose.position.z + 0.05+ dist_z
    pose7.pose.orientation.x = pose.pose.orientation.x
    pose7.pose.orientation.y = pose.pose.orientation.y
    pose7.pose.orientation.z = pose.pose.orientation.z
    pose7.pose.orientation.w = pose.pose.orientation.w
    m7 = ik.get_ik(pose7,robot.get_current_state())
    m7 = m7[7:13]
    m7.append(0)
    print "ik 7",m7
    rospy.sleep(0.5)
    # 8: simplified rotate's ik
    pose8 = PoseStamped()
    pose8.header.frame_id = "world"
    pose8.header.stamp = rospy.Time.now()
    pose8.pose.position.x = pose.pose.position.x+ dist_x
    pose8.pose.position.y = pose.pose.position.y+ dist_y
    pose8.pose.position.z = pose.pose.position.z + 0.02+ dist_z
    pose8.pose.orientation.x = pose.pose.orientation.x
    pose8.pose.orientation.y = pose.pose.orientation.y
    pose8.pose.orientation.z = pose.pose.orientation.z
    pose8.pose.orientation.w = pose.pose.orientation.w
    m8 = ik.get_ik(pose8,robot.get_current_state())
    m8 = m8[7:13]
    m8.append(0)
    print "ik 8",m8
    rospy.sleep(0.5)
    k_start = [m]
    # k_start = [m,m2]
    return k_start

def flexflip(ref_tag,trans_target2ref,target_rot_angle,trans_fix2ref,fix_rot_angle,crease_axis,crease_perp_l,margin,tag_pos,tag_rot):
    ##IN CONVENTION: crease_axis and perp_axis have negative y axis value for tag 15 and tag 17
    ##               crease_axis and perp_axis have positive y axis value for tag 27 and tag 29

    if crease_axis[0]*crease_axis[1]>=0:
        sign=-1
    elif crease_axis[0]*crease_axis[1]<0:
        sign=1

    if crease_axis[1]>=0:
        ori=-1
    elif crease_axis[1]<0:
        ori=1

    z=[0,0,1]
    perp_axis=np.cross(z,crease_axis)  #perp_axis=[-0.7071,-0.7071,0]
    rot1_angle = 15*sign                    #15
    rot2_angle = 20*sign                     #20
    rot3_angle = -45*sign                      #-45
    rot4_angle = -30*sign                      #-25

    crease_perp_l=crease_perp_l*ori
    margin_x=-perp_axis[0]*margin[0]*sign
    margin_y=-perp_axis[1]*margin[1]*sign

    print "============ perp_axis is %s" % perp_axis
    ########## flexflip starts here
    print "============= Start flexflip"
    ############## init grippers
    arduino_pub = rospy.Publisher('/hybrid', UInt16, queue_size=1)
    rospy.sleep(1)
    arduino_pub.publish(201)

    #####Obtain key info of tags
    # listener.waitForTransform("world", ref_tag, rospy.Time(), rospy.Duration(4.0))
    # (trans_ref,rot_) = listener.lookupTransform("world", ref_tag, rospy.Time(0))
    trans_ref = [0.05,0.05,0.71]

    ##############kong_arm to initial pose
    phi=30
    listener.waitForTransform("world", "soft_tip_link2", rospy.Time(), rospy.Duration(4.0))
    (trans_soft,rot_) = listener.lookupTransform("world", "soft_tip_link2", rospy.Time(0))
    group_rotate_by_external_axis(trans_soft, [0, 1.0, 0], phi,"robkong")
    phi = target_rot_angle
    listener.waitForTransform("world", "soft_tip_link2", rospy.Time(), rospy.Duration(4.0))
    (trans_soft,rot_) = listener.lookupTransform("world", "soft_tip_link2", rospy.Time(0))
    group_rotate_by_external_axis(trans_soft, [0, 0, 1.0], phi,"robkong")

    listener.waitForTransform("world", "soft_tip_link2", rospy.Time(), rospy.Duration(4.0))
    (trans_soft,rot_) = listener.lookupTransform("world", "soft_tip_link2", rospy.Time(0))
    #move to top of target tag
    trans_target= np.add(trans_target2ref,trans_ref).tolist()
    trans_soft2targ = np.subtract(trans_target,trans_soft).tolist()
    trans_fix= np.add(trans_fix2ref,trans_ref).tolist()
    print "============ trans_ref is %s" % trans_ref
    print "============ trans_target is %s" % trans_target
    print "============ trans_fix is %s" % trans_fix


    move_waypoints(trans_soft2targ[0]+margin_x,trans_soft2targ[1]+margin_y,0.00,1,'robkong')
#     kong_arm descend
    descend_to_desktop('robkong','normal',0.004+margin[2])
    group2_pose = group2.get_current_pose().pose
    print "============ current pose is %s" % group2.get_current_pose().pose
    print "============ current joint value is %s" % group2.get_current_joint_values()

    ###################################### start to fix the end via hong_arm/robhong/group1
    k_start = infer_ik(group2_pose,perp_axis,crease_perp_l)
    h_start = group1.get_current_joint_values()
    h_start = h_start[:6]
    h_start.append(0)
    fix(k_start,h_start,tag_pos,tag_rot)
    # fix(trans_fix,fix_rot_angle,'robhong')

    ####################################### flex-flip and grasp
    ######
    arduino_pub.publish(202)
    rospy.sleep(4)

    move_waypoints(0,0,0.01,1,'robkong')
    rospy.sleep(1)
    listener.waitForTransform("world", "soft_tip_link2", rospy.Time(), rospy.Duration(4.0))
    (trans_soft,rot_) = listener.lookupTransform("world", "soft_tip_link2", rospy.Time(0))
    #rot1: to hold the paper after flexflip
    group_rotate_by_external_axis(trans_soft, crease_axis, rot1_angle,'robkong')
    #pinch!
    rospy.sleep(1)
    arduino_pub.publish(203)
    rospy.sleep(2)
    move_waypoints(0.0,0.0,0.01,1,'robkong')
#
#
# # # #     #rot2: make pinch tip vertical
#
#     fold(crease_axis,rot2_angle,'robkong')
#
#     listener.waitForTransform("world", "pinch_tip_kong", rospy.Time(), rospy.Duration(4.0))
#     (trans_pinch,rot_) = listener.lookupTransform("world", "pinch_tip_kong", rospy.Time(0))
#     delta_z=-trans_pinch[2]+0.708+0.05
#     move_waypoints(0,0,delta_z,1,'robkong')
# #     #rot3:rot pinch tip for crease
#     fold(crease_axis,rot3_angle,'robkong')
#     move_along_axis(perp_axis,-crease_perp_l,'robkong')
#
#     listener.waitForTransform("world", "pinch_tip_kong", rospy.Time(), rospy.Duration(4.0))
#     (trans_pinch,rot_) = listener.lookupTransform("world", "pinch_tip_kong", rospy.Time(0))
#     delta_z=-trans_pinch[2]+0.703
#     move_waypoints(0,0,delta_z,1,'robkong')
#
#     ###let arm fixing the paper go home
#     robhong_go_to_home(1)
#     #rot4: rot more to leave space for other robot arm
#     fold(crease_axis,rot4_angle,'robkong')
#     # grasp the paper tip tighter
#     arduino_pub.publish(212)

def crease(trans_ref,pinch_z_angle,crease_axis,crease_length,startP2refP,z_axis,margin_z,margin_offset):

    ##IN CONVENTION: crease_axis and normal_axis have negative y axis value
    if crease_axis[1]>=0:
        ori=-1
    elif crease_axis[1]<0:
        ori=1

    normal_axis=np.cross(z_axis,crease_axis)  #perp_axis=[-0.7071,-0.7071,0] default z_axis=[0,0,1]

    offset_unit=-(0.015+margin_offset)*ori
    shift_unit=0.02

    # make sure hong is at home
    robhong_go_to_home(1)
    # scoop uses single arm, currently hong_arm/robhong/group1
    print "============ Start scooping"
    # set gripper_hong's rigid finger to pinch state
    arduino_pub = rospy.Publisher('/hybrid', UInt16, queue_size=1)
    rospy.sleep(1)
    arduino_pub.publish(121)

    ##############hong_arm to start pose
    listener.waitForTransform("world", "pinch_tip_hong", rospy.Time(), rospy.Duration(4.0))
    (trans_pinchH,rot_) = listener.lookupTransform("world", "pinch_tip_hong", rospy.Time(0))


    phi=pinch_z_angle
    group_rotate_by_external_axis(trans_pinchH, [0, 0, 1], phi,"robhong")

    listener.waitForTransform("world", "pinch_tip_hong", rospy.Time(), rospy.Duration(4.0))
    (trans_pinchH,rot_) = listener.lookupTransform("world", "pinch_tip_hong", rospy.Time(0))
    trans_pinch2tag = np.subtract(trans_ref,trans_pinchH).tolist()
    trans_pinch2start = np.add(startP2refP,trans_pinch2tag).tolist()
    #move to scooping x and y pose
    move_waypoints(trans_pinch2start[0],trans_pinch2start[1],0.0,1,'robhong')
    #hong_arm descend
    descend_to_desktop('robhong','pinch',margin_z)

    ##################  start to make a crease along defined axis
    pinch_dist=[normal_axis[0]*offset_unit,normal_axis[1]*offset_unit,normal_axis[2]*offset_unit]
    shift_dist=[crease_axis[0]*shift_unit,crease_axis[1]*shift_unit,crease_axis[2]*shift_unit]
    num_attempts=int(crease_length//shift_unit)
    print "====pinch attempts"
    print num_attempts
    for i in range(0,num_attempts):
        listener.waitForTransform("world", "pinch_tip_hong", rospy.Time(), rospy.Duration(4.0))
        (trans_pH,rot_) = listener.lookupTransform("world", "pinch_tip_hong", rospy.Time(0))
        (trans_pK,rot_) = listener.lookupTransform("world", "pinch_tip_kong", rospy.Time(0))
        trans_delta=np.subtract(trans_pK,trans_pH).tolist()
        print "======== trans_pH", trans_pH
        print "======== trans_pK", trans_pK
        print "======== trans_delta", trans_delta
        dist=sqrt(trans_delta[0]**2+trans_delta[1]**2)
        print "======== dist between tips ", dist
        if dist>=0.024:
            move_waypoints(pinch_dist[0],pinch_dist[1],pinch_dist[2],0.2,"robhong")
            arduino_pub.publish(112)
            rospy.sleep(6)
            arduino_pub.publish(111)
            rospy.sleep(3)
            move_waypoints(-pinch_dist[0],-pinch_dist[1],-pinch_dist[2],0.2,"robhong")
        else:
            rospy.sleep(3)
    move_waypoints(shift_dist[0],shift_dist[1],shift_dist[2],0.2,"robhong")

    move_waypoints(pinch_dist[0],pinch_dist[1],pinch_dist[2],0.2,"robhong")
    arduino_pub.publish(112)
    rospy.sleep(6)
    arduino_pub.publish(111)
    rospy.sleep(2)

    move_waypoints(shift_dist[0],shift_dist[1],shift_dist[2],0.2,"robhong")

    robhong_go_to_home(1)
    arduino_pub.publish(211)
    robkong_go_to_home(1)

    arduino_pub.publish(1)
    rospy.sleep(2)
#####################initialization#########################
def init():
    group1.clear_pose_targets()
    group2.clear_pose_targets()
    planning_frame = group1.get_planning_frame()
    print "============ Reference frame1: %s" % planning_frame
    planning_frame = group2.get_planning_frame()
    print "============ Reference frame2: %s" % planning_frame
    print "============ Robot Groups:", robot.get_group_names()

    robkong_go_to_home(1)
    robhong_go_to_home(1)

    # arduino_pub = rospy.Publisher('/hybrid', UInt16, queue_size=1)
    # rospy.sleep(1)
    # arduino_pub.publish(1)
# scene.remove_world_object(paper_name)
#####################initialization#########################

#####################generate fix configuration#######################
init()
# step1: get apriltag transformation
# (trans_tag,rot_tag) = listener.lookupTransform("world", "tag_22", rospy.Time(0))
# tag_pos = [trans_tag[0],trans_tag[1],0.71]
# tag_rot_mat = tf.transformations.quaternion_matrix(rot_tag)
# tag_rot = tag_rot_mat[:2,:2]
tag_pos = [0.05,0.05,0.71]
tag_rot = [[1,0],[0,1]]
print "tag_pos",tag_pos
print "tag_rot",tag_rot
points = hf.paper_polygen(tag_pos,tag_rot)
for i in range(len(points)):
    showpoint(points[i],"p"+str(i),1)
    rospy.sleep(1)
# step2: flex flip
crease_axis= [-0.7071, -0.7071, 0]
crease_perp_l=0.027
crease_length=0.14
startP2refP=[0.10,-0.065,0] # in world frame

trans_target2ref=[0.10,-0.15,0]  # in world frame
trans_fix2ref=[0.09,0.14,0]    # in world frame

# listener.waitForTransform("world", "tag_22", rospy.Time(), rospy.Duration(4.0))
# (trans_ref,rot_) = listener.lookupTransform("world", 'tag_22', rospy.Time(0))

flexflip('tag_22',trans_target2ref, -45.0, trans_fix2ref, -135.0,crease_axis, crease_perp_l,[0.0,0.0,0],tag_pos,tag_rot)

# crease(trans_ref,-45.0,crease_axis,crease_length,startP2refP,z_axis=[0,0,1],margin_z=0.005,margin_offset=0.005)
