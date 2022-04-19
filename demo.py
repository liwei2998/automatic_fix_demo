from __future__ import division
import pybullet as p
import pybullet_data
import numpy as np
import time
import argparse
import pybullet_utils

UR10_JOINT_INDICES = [0, 1, 2, 3, 4, 5, 8] #movable joints of ur10_hong


def set_joint_positions(body, joints, values):
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        p.resetJointState(body, joint, value)


def draw_sphere_marker(position, radius, color):
    vs_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
    marker_id = p.createMultiBody(basePosition=position, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vs_id)
    return marker_id


def remove_marker(marker_id):
    p.removeBody(marker_id)


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--birrt', action='store_true', default=False)
    parser.add_argument('--smoothing', action='store_true', default=False)
    args = parser.parse_args()
    return args


def rrt():
    ###############################################
    # TODO your code to implement the rrt algorithm
    ###############################################
    pass


def birrt():
    #################################################
    # TODO your code to implement the birrt algorithm
    #################################################
    pass


def birrt_smoothing():
    ################################################################
    # TODO your code to implement the birrt algorithm with smoothing
    ################################################################
    pass


if __name__ == "__main__":
    args = get_args()

    # set up simulator
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.setGravity(0, 0, -9.8)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, True)
    p.resetDebugVisualizerCamera(cameraDistance=1.400, cameraYaw=58.000, cameraPitch=-42.200, cameraTargetPosition=(0.0, 0.0, 0.0))
    CLIENT = 0
    # load objects
    plane = p.loadURDF("plane.urdf")
    ur10_hong = p.loadURDF('assets/ur5/ur10_hong_gripper.urdf', basePosition=[0.885, 0.012, 0.786], baseOrientation=[0,0,0.70710678,0.70710678], useFixedBase=True)
    ur10_kong = p.loadURDF('assets/ur5/ur10_kong_gripper.urdf', basePosition=[-0.916, 0, 0.77], baseOrientation=[0,0,-0.70710678,0.70710678],useFixedBase=True)
    obstacle1 = p.loadURDF('assets/ur5/robot_optical_table_hong.urdf',
                           basePosition=[1.35, 0.012, 0.016],
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

    # start and goal
    start_conf_kong = (-1.24, -2.23, -1.6,-1.13,1.46,1.13,-0.47)
    start_conf_hong = (0.96, -1.07, 1.37,-1.31,-1.7,0.32,0.7)
    # start_conf_hong = (1.19, -0.88, 1.37,-1.31,-1.7,0.32,0.7) #collision test
    start_position = (0.3998897969722748, -0.3993956744670868, 0.6173484325408936)
    goal_conf = (0.7527214782907734, -0.6521867735052328, -0.4949270744967443,0,0,0)
    goal_position = (0.35317009687423706, 0.35294029116630554, 0.7246701717376709)
    goal_marker = draw_sphere_marker(position=goal_position, radius=0.02, color=[1, 0, 0, 1])
    set_joint_positions(ur10_hong, UR10_JOINT_INDICES, start_conf_hong)
    set_joint_positions(ur10_kong, UR10_JOINT_INDICES, start_conf_kong)
    # place holder to save the solution path
    path_conf = None

    # get the collision checking function
    from collision_utils import get_collision_fn
    from collision_utils import *
    collision_fn = get_collision_fn(ur10_hong, UR10_JOINT_INDICES, obstacles=obstacles,
                                    attachments=[], self_collisions=True,
                                    disabled_collisions=set())
    print "collision",collision_fn(start_conf_hong)
    jointt= get_joint(ur10_hong,1)
    print 'joint',jointt
    if args.birrt:
        if args.smoothing:
            # using birrt with smoothing
            path_conf = birrt_smoothing()
        else:
            # using birrt without smoothing
            path_conf = birrt()
    else:
        # using rrt
        path_conf = rrt()

    if path_conf is None:
        # pause here
        raw_input("no collision-free path is found within the time budget, finish?")
    else:
        ###############################################
        # TODO your code to highlight the solution path
        ###############################################

        # execute the path
        while True:
            for q in path_conf:
                #set_joint_positions(ur5, UR5_JOINT_INDICES, q)
                time.sleep(0.5)
