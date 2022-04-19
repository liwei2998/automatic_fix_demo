#!/usr/bin/env python

import rospy
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.srv import GetPositionIKRequest
from moveit_msgs.srv import GetPositionIKResponse

class GetIK(object):
    def __init__(self, group, ik_timeout=10.0, ik_attempts=0, avoid_collisions=True):
        """
        A class to do IK calls thru the MoveIt!'s /compute_ik service.
        :param str group: MoveIt! group name
        :param float ik_timeout: default timeout for IK
        :param int ik_attempts: default number of attempts
        :param bool avoid_collisions: if to ask for IKs that take
        into account collisions
        """
        self.group_name = group
        self.ik_timeout = ik_timeout
        self.ik_attempts = ik_attempts
        self.avoid_collisions = avoid_collisions
        self.ik_srv = rospy.ServiceProxy('/compute_ik', GetPositionIK)
        self.ik_srv.wait_for_service()

    def get_ik(self, pose_stamped,
            robot_state,
            group=None,
            ik_timeout=None,
            ik_attempts=None,
            avoid_collisions=None):
        """
        Do an IK call to pose_stamped pose.
        :param geometry_msgs/PoseStamped pose_stamped: The 3D pose
            (with header.frame_id)
            to which compute the IK.
        :param str group: The MoveIt! group.
        :param float ik_timeout: The timeout for the IK call.
        :param int ik_attemps: The maximum # of attemps for the IK.
        :param bool avoid_collisions: If to compute collision aware IK.
        """
        if group is None:
            group = self.group_name
        if ik_timeout is None:
            ik_timeout = self.ik_timeout
        if ik_attempts is None:
            ik_attempts = self.ik_attempts
        if avoid_collisions is None:
            avoid_collisions = self.avoid_collisions
        req = GetPositionIKRequest()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state = robot_state
        req.ik_request.pose_stamped = pose_stamped
        # req.ik_request.ik_link_name = "rigid_tip_link1"
        req.ik_request.timeout = rospy.Duration(ik_timeout)
        req.ik_request.attempts = ik_attempts
        req.ik_request.avoid_collisions = avoid_collisions

        try:
            resp = self.ik_srv.call(req)
            # resp = list(resp.solution.joint_state.position[7:13])
            resp = list(resp.solution.joint_state.position)
            return resp
        except rospy.ServiceException as e:
            rospy.logerr("Service exception: " + str(e))
            resp = GetPositionIKResponse()
            resp.error_code = 99999  # Failure
            return resp
