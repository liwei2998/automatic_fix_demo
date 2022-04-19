#!/usr/bin/env python
from __future__ import division
import sys
import os
import get_ik
import rospy
import time
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
import random
from math import pi
import matplotlib.pyplot as plt
from shapely.geometry import *
from std_msgs.msg import *
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionFKRequest
from moveit_msgs.srv import GetPositionFKResponse
import pybullet as p
import pybullet_data
import pybullet_utils
from collision_utils import get_collision_fn
import pybullet_planning as pp
from scipy import interpolate
import scipy.io as sio
import IPython

UR10_JOINT_INDICES = [0, 1, 2, 3, 4, 5, 8] #movable joints of ur10_hong

def set_joint_positions(body, joints, values):
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        p.resetJointState(body, joint, value)

def is_inPoly(polygen,points):
    # this is to determine is a point is in a polygen
    line = LineString(polygen)
    point = Point(points)
    polygen = Polygon(line)
    return polygen.contains(point)

def get_poly_from_gripper(center):
    #this is to get the gripper polygen of a chosen robot arm
    # center = group.get_current_pose().pose
    c_pos = np.array([center.position.x,center.position.y,center.position.z])
    c_ori = [center.orientation.x,center.orientation.y,center.orientation.z,center.orientation.w]
    c_mat = tf.transformations.quaternion_matrix(c_ori)
    c_mat = c_mat.transpose()
    x_axis = c_mat[0]
    x_axis = x_axis[:3]
    x_axis[2] = 0
    y_axis = c_mat[1]
    y_axis = y_axis[:3]
    y_axis[2] = 0
    x_axis = x_axis / np.linalg.norm(x_axis)
    y_axis = y_axis / np.linalg.norm(y_axis)
    p1 = c_pos + 0.04*x_axis + 0.08*y_axis
    p2 = c_pos + 0.04*x_axis - 0.08*y_axis
    p3 = c_pos - 0.04*x_axis - 0.08*y_axis
    p4 = c_pos - 0.04*x_axis + 0.08*y_axis
    return p1, p2, p3, p4

def weight_sample_x_y(a1 = np.arange(-0.105,0,0.0001),
                      a2 = np.arange(0,0.105,0.0001),
                      b1 = np.arange(0.1,0.145,0.0001),
                      b2 = np.arange(0.02,0.1,0.0001),
                      b3 = np.arange(-0.05,0.02,0.0001),
                      lst_a = [0.5,0.75],
                      lst_b = [0.85,0.15,0.05]):
    # weighted sample, to make the sample more efficiently
    a = [a1,a2]
    b = [b1, b2, b3]

    def w_sample(lst):
        w_total = sum(x for x in lst)
        n = random.uniform(0,w_total)
        for i, weight in enumerate(lst):
            if n < weight:
                break
            n = n - weight
        return i
    sample_a = a[w_sample(lst_a)]
    sample_b = b[w_sample(lst_b)]
    x = random.choice(sample_a)
    y = random.choice(sample_b)
    return x, y

def plane_transformation(x,y,tag_pos,tag_rot):
    #used for apriltag frame transformation
    new = np.dot(tag_rot,np.array([x,y])) + tag_pos[:2]
    return new[0],new[1]

def paper_polygen(tag_pos,tag_rot):
    #used to get paper vertice positions in real world
    #return vertice positions through apriltag information
    v1 = np.array([0.105,0.145,0.71])
    v2 = np.array([0.105,-0.145,0.71])
    v3 = np.array([-0.105,-0.145,0.71])
    v4 = np.array([-0.105,0.145,0.71])
    v = [v1,v2,v3,v4]
    new_v = []
    for i in range(len(v)):
        x,y = plane_transformation(v[i][0],v[i][1],tag_pos,tag_rot)
        x,y =round(x,3),round(y,3)
        new_v.append([x,y,0.71])
    return new_v

def config_generate(group,center,kong_configs,robot_state,tag_pos,tag_rot,num=5,sim=True):
    # get collision free ik solutions from random sample.
    # solutions are stored in conf[]
    #step1: set up simulator
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.setGravity(0, 0, -9.8)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, False)
    p.resetDebugVisualizerCamera(cameraDistance=1.400, cameraYaw=58.000, cameraPitch=-42.200, cameraTargetPosition=(0.0, 0.0, 0.0))
    # load objects
    plane = p.loadURDF("plane.urdf")
    if sim == 1:
        baseOrientation1 = [0,0,0.70710678,0.70710678]
        baseOrientation2 = [0,0,-0.70710678,0.70710678]
    elif sim == 0:
        baseOrientation1 = [0,0,-0.70710678,0.70710678]
        baseOrientation2 = [0,0,0.70710678,0.70710678]

    ur10_hong = p.loadURDF('assets/ur5/ur10_hong_gripper.urdf', basePosition=[0.885, 0.012, 0.786], baseOrientation=baseOrientation1, useFixedBase=True)
    ur10_kong = p.loadURDF('assets/ur5/ur10_kong_gripper.urdf', basePosition=[-0.916, 0, 0.77], baseOrientation=baseOrientation2,useFixedBase=True)
    obstacle1 = p.loadURDF('assets/ur5/robot_optical_table_hong.urdf',
                           basePosition=[1.35,0.012,0.016],
                           baseOrientation=[0,0,1,0],
                           useFixedBase=True)
    obstacle2 = p.loadURDF('assets/ur5/robot_movable_table.urdf',
                           basePosition=[0, 0, 0],
                           baseOrientation=[0,0,0,1],
                           useFixedBase=True)
    obstacle3 = p.loadURDF('assets/ur5/robot_optical_table_kong.urdf',
                           basePosition=[-1.381, 0, 0],
                           baseOrientation=[0,0,1,0],
                           useFixedBase=True)
    obstacles = [plane, ur10_kong, obstacle2, obstacle3]
    # visualize paper vertices, reference:
    # https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/createVisualShape.py
    visualShapeId = p.createVisualShape(shapeType=p.GEOM_BOX,
                                    halfExtents = [0.01,0.01,0.01],
                                    rgbaColor=[1, 1, 1, 1],
                                    specularColor=[0.4, .4, 0],
                                    visualFramePosition=[0,0,0])

    #step2: set start configuration
    start_conf_kong = kong_configs[0]
    set_joint_positions(ur10_kong, UR10_JOINT_INDICES, start_conf_kong)
    #step3: randomly sample configs
    ik = get_ik.GetIK(group)
    pose = PoseStamped()
    pose.header.frame_id = "world"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.z = 0.715 + 0.1411 #0.71(table height) + 0.11411 (dis_z between rogid_tip_link and hong_tool0)
    # polygen = get_poly_from_gripper(center)
    c = np.arange(-np.pi,np.pi,np.pi/1000)
    conf = []
    i = 0
    # get paper vertices with apriltag information
    poly = paper_polygen(tag_pos,tag_rot)
    # show paper vertices in pybullet
    for ver in range(len(poly)):
        p.createMultiBody(baseMass=1,
                          baseInertialFramePosition=[0, 0, 0],
                          baseVisualShapeIndex=visualShapeId,
                          basePosition=poly[ver],
                          useMaximalCoordinates=True)
    rospy.sleep(1)
    while (i<num):
        # sample orientation
        theta = random.choice(c)
        # sample position
        x, y = weight_sample_x_y() #weighted sample
        x, y = plane_transformation(x,y,tag_pos,tag_rot) #transform with apriltag information
        rospy.sleep(0.5)
        #we have sampled rigid_link_tip1's position, and now transform this position to hong_tool0's position, to compute ik
        mat = [[np.cos(theta),np.sin(theta),0,0],[np.sin(theta),-np.cos(theta),0,0],[0,0,-1,0],[0,0,0,1]]
        trans = np.array([-0.07200367743271228, 0.059549999999985115, -0.1411026781906094, 0])
        delta = np.dot(np.array(mat),trans)
        pose.pose.position.x = x + delta[0]
        pose.pose.position.y = y + delta[1]
        #
        # if is_inPoly(polygen,(x,y,0.87)) == 1:
        #     continue
        ori = tf.transformations.quaternion_from_matrix(mat)
        pose.pose.orientation.x = ori[0]
        pose.pose.orientation.y = ori[1]
        pose.pose.orientation.z = ori[2]
        pose.pose.orientation.w = ori[3]
        # compute ik
        m = ik.get_ik(pose,robot_state)
        if m == []:
            print "no configs"
            continue
        #to make sure that rigid_link_tip1 is in the paper
        elif test_fk(m[:6],poly,robot_state) == 0:
            print "not in paper polygen"
            continue
        else:
            #step3: collision check
            # set hong's config first as m
            set_joint_positions(ur10_hong, UR10_JOINT_INDICES, m[:7])
            count = 0
            # print 'm0',m
            # for every configs in kong's trajectory, conduct collision check
            for aa in range(len(kong_configs)):
                set_joint_positions(ur10_kong, UR10_JOINT_INDICES, kong_configs[aa])
                # reference: from collision_utils import get_collision_fn
                collision_fn = get_collision_fn(ur10_hong, UR10_JOINT_INDICES, obstacles=obstacles,
                                                attachments=[], self_collisions=True,
                                                disabled_collisions=set())
                # if collision_fn(m[:7]) == 0, no collision
                print 'collision?',collision_fn(m[:7])
                if collision_fn(m[:7]) is None:
                    count = count + 1
                else:
                    break
            # if count == len(kong_configs), the config of hong is collision-free with all configs in kong's trajectory
            if count == len(kong_configs):
                # for bb in range(7):
                #     if m[bb] >= 3.1415926:
                #         m[bb] = m[bb] - 3.1415926
                #     elif m[bb] <= -3.1415926:
                #         m[bb] = m[bb] + 3.1415926
                print "m",m
                conf.append(m)
                i = i +1
    conf = np.array(conf)
    conf = conf[:,:7]
    conf = conf.tolist()
    return conf

def test_fk(config,paper_polygen,robot_state):
    #test if rigid_tip_link1 is in the paper polygen
    ik_srv = rospy.ServiceProxy('/compute_fk', GetPositionFK)
    ik_srv.wait_for_service()

    header = Header()
    header.frame_id = "world"
    header.stamp = rospy.Time.now()
    fk_link_names = ["rigid_tip_link1"]
    # robot_state = robot.get_current_state()
    # 0: rigid_rev_joint1 and 2.
    # -1.75...: kong's joint values
    # the values of other_joint_values does not affect the result
    other_joint_values = [0, -1.7504954265803478, -1.9503007193486717, -1.340203426021494,
                          -1.40115032350114, 1.6977166700000357, 0.21991148575129937, 0]

    robot_state.joint_state.position = config + other_joint_values
    resp = ik_srv(header,fk_link_names,robot_state)
    pos = [resp.pose_stamped[0].pose.position.x,resp.pose_stamped[0].pose.position.y,
           resp.pose_stamped[0].pose.position.z]

    def is_inPoly(paper_polygen,pos):
        line = LineString(paper_polygen)
        point = Point(pos)
        polygen = Polygon(line)
        return polygen.contains(point)

    return is_inPoly(paper_polygen,pos)

def get_dis(configs,k_start_config,h_start_config,connect_mode=p.GUI):
    #input valid configs of hong, obtain smallest distance for each config
    #step1: set up simulator
    physicsClient = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.setGravity(0, 0, -9.8)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, True)
    p.resetDebugVisualizerCamera(cameraDistance=1.400, cameraYaw=58.000, cameraPitch=-42.200, cameraTargetPosition=(0.0, 0.0, 0.0))
    # load objects
    plane = p.loadURDF("plane.urdf")
    ur10_hong = p.loadURDF('assets/ur5/ur10_hong_gripper.urdf', basePosition=[0.885, 0.012, 0.786], baseOrientation=[0,0,0.70710678,0.70710678], useFixedBase=True)
    ur10_kong = p.loadURDF('assets/ur5/ur10_kong_gripper.urdf', basePosition=[-0.916, 0, 0.77], baseOrientation=[0,0,-0.70710678,0.70710678],useFixedBase=True)
    obstacle2 = p.loadURDF('assets/ur5/robot_movable_table.urdf',
                           basePosition=[0, 0, 0],
                           baseOrientation=[0,0,0,1],
                           useFixedBase=True)
    obstacle3 = p.loadURDF('assets/ur5/robot_optical_table_kong.urdf',
                           basePosition=[-1.381, 0, 0],
                           baseOrientation=[0,0,1,0],
                           useFixedBase=True)
    obstacles = [plane, ur10_kong, obstacle2, obstacle3]
    #step2: set start configs
    start_conf_kong = k_start_config[0]
    start_conf_hong = h_start_config
    set_joint_positions(ur10_hong, UR10_JOINT_INDICES, start_conf_hong)
    set_joint_positions(ur10_kong, UR10_JOINT_INDICES, start_conf_kong)
    #step3: get min dis list
    min_dis = []
    for c in configs:
        goal_conf_hong = c
        #reference: import pybullet_planning as pp
        #return path: path planed by birrt algorithm
        #return dis: closest distances between 2 arms in each collision check along the path. return a list
        path, dis = pp.plan_joint_motion(ur10_hong,UR10_JOINT_INDICES,start_conf_hong, goal_conf_hong,obstacles)
        #if dis is none, the path is invalid, continue
        if dis is None:
            min_dis.append(0)
            continue
        min_dis_tmp = 0
        # is the path is valid,
        w1 = 0.5 / len(dis)
        w2 = 0.5
        # get closest distance between 2 arms at the final configuration. return a list information
        final_dis = p.getClosestPoints(bodyA=ur10_hong, bodyB=ur10_kong, distance=100000,
                                      linkIndexA=9,physicsClientId=0)
        final_dis = np.array(final_dis)
        # choose the distance information from the return. now final_dis is the closest distance, a float
        final_dis = np.min(final_dis[:,8])
        # weighted add closest distance
        for i in range(len(dis)):
            min_dis_tmp += w1 * dis[i]
        min_dis_tmp += w2 * final_dis
        # now min_dis_tmp can represent the closest distance along the trajectory
        # min_dis is a list that restore min_dis_tmp along all trajectorys
        min_dis.append(min_dis_tmp)
    return min_dis
