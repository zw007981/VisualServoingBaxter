#!/usr/bin/env python
import sys
from copy import copy
import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)
import baxter_interface
import numpy
import math
from moveit_commander import conversions
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest


class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in
                                             ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]


def transform(rpy_pose):
    limb = 'left'
    node = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    ik_service = rospy.ServiceProxy(node, SolvePositionIK)
    quaternion_pose = conversions.list_to_pose_stamped(rpy_pose, "base")
    ik_request = SolvePositionIKRequest()
    ik_request.pose_stamp.append(quaternion_pose)
    ik_response = ik_service(ik_request)
    # limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position))
    # ik_response.joints[0].position = [s0, s1, e0, e1, w0, w1, w2], this is also the target point we want.
    return(ik_response.joints[0].position)


def cal_graident(alphab, alphad, betag, beta, dg, d):
    betag_norm = numpy.sqrt(numpy.dot(betag.T, betag)[0, 0])
    betag_new = betag / betag_norm
    beta_norm = numpy.sqrt(numpy.dot(beta.T, beta)[0, 0])
    beta_new = beta / beta_norm
    gb = betag_new - beta_new
    gd = dg * betag_new - numpy.dot(d * beta_new.T, betag_new) * betag_new
    phi = alphab * gb + alphad * gd
    return(phi)


def modify(circle_center, r_, x_ball, y_ball):
    len = circle_center.shape[1]
    flag = 0
    # if flag == 0, no collision.
    phi_ = numpy.array([phi[0], phi[1]])
    x = numpy.array([x[0], x[1]])
    for i in range(0, len):
        center = circle_center[:, i]
        center = numpy.array([center[0], center[1]])
        r = r_[i]
        v1 = numpy.array[[x_ball, y_ball]] - center
        d = numpy.sqrt(numpy.dot(v1.T, v1))[0, 0]
        if d[0, 0] <= 1.01 * r:
            flag = 1
            break
        # d = (numpy.dot(v1, phi_)) / numpy.sqrt(numpy.dot(phi_.T, phi_)[0, 0])

    if flag == 1:
        z_modify = 0.4
    else:
        z_modify = 0.3
    return(z_modify)


def main():
    z_initial = 0.3
    # z_initial is the initial height.
    t1 = 3
    # t1 is the time baxter needs to excute requested joint trajectory.
    rpy_pose = [0.652, 0.064, z_initial, -1.0 * math.pi, 0, 0]
    # rpy_pose = [self.x, self.y, self.z, self.roll, self.pitch, self.yaw],
    # this is the initial position we want move to.
    limb = 'left'
    [alphab, alphad, dg] = [0.48, 0.48, 0.3]
    betag = numpy.array([[0], [0], [1]])
    circle_center = numpy.array([[0.52], [0.04], [0.16]])
    r_ = numpy.array([0.20])

    rospy.init_node("rsdk_joint_trajectory_client_%s" % (limb,))
    traj = Trajectory(limb)
    rospy.on_shutdown(traj.stop)
    limb_interface = baxter_interface.limb.Limb(limb)
    current_angles = [limb_interface.joint_angle(
        joint) for joint in limb_interface.joint_names()]
    traj.add_point(current_angles, 0.0)
    pose = transform(rpy_pose)
    traj.add_point(pose, 2 * t1)
    traj.start()
    traj.wait(10 * t1 + 3)
    traj.clear(limb)
    # move to the initial position.

    while 1:
        ball = numpy.loadtxt("/home/billierose/code/ros_ws/record.txt")
        l_ball = len(ball)
        x_ball = ball[l_ball - 2]
        y_ball = ball[l_ball - 1]
        z_ball = -0.07
        # position of the object.
        camera = numpy.loadtxt("/home/billierose/code/ros_ws/record1.txt")
        l_camera = len(camera)
        x_camera = camera[l_camera - 2]
        y_camera = camera[l_camera - 1]
        z_camera = z_initial
        # location of the end of baxter arm.
        vector = numpy.array(
            [[x_ball - x_camera], [y_ball - y_camera], [z_ball - z_camera]])
        vector_norm = numpy.sqrt(numpy.dot(vector.T, vector)[0, 0])
        beta = vector / vector_norm
        phi = cal_graident(alphab, alphad, betag, beta, dg, vector_norm)
        #z_modify = modify(circle_center, r_, x_ball, y_ball)
        x_object = x_camera - phi[0, 0]
        y_object = y_camera - phi[1, 0]
        z_object = z_initial

        limb_interface = baxter_interface.limb.Limb(limb)
        current_angles = [limb_interface.joint_angle(
            joint) for joint in limb_interface.joint_names()]
        traj.add_point(current_angles, 0.0)

        x_ballp = ball[l_ball - 4]
        y_ballp = ball[l_ball - 3]
        deltax = x_ball - x_ballp
        deltay = y_ball - y_ballp
        zw = numpy.sqrt(deltax**2 + deltay**2)

        rpy_pose = [x_object, y_object, z_object, -1.0 * math.pi, 0, 0]
        pose = transform(rpy_pose)
        traj.add_point(pose, t1)
        if zw >= 0.0001:
            traj.start()
            traj.wait(t1 + 1.2)
        traj.clear(limb)


if __name__ == "__main__":
    main()
