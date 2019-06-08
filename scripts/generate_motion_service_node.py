#!/usr/bin/env python
import rospy
import numpy as np
from ros_dmp import RollDmp
from ros_dmp.srv import GenerateMotion, GenerateMotionResponse
from ros_dmp.msg import CartesianTrajectory
from nav_msgs.msg import Path
import tf


class GenerateMotionClass:

    def __init__(self):

        rospy.init_node("generate_motion_service_node")
        rospy.Service("generate_motion_service", GenerateMotion, self.generate_motion)
        rospy.loginfo("Started Motion Generation Service")
        # Publishers
        self.trajectory_pub = rospy.Publisher('~cartesian_trajectory', CartesianTrajectory, queue_size=1)
        self.path_pub = rospy.Publisher('~cartesian_path', Path, queue_size=1)

    def generate_motion(self, req):

        rospy.loginfo("Received motion generation request")
        # Initial pose
        rpy = tf.transformations.euler_from_quaternion([
                req.initial_pose.pose.orientation.x, req.initial_pose.pose.orientation.y,
                req.initial_pose.pose.orientation.z, req.initial_pose.pose.orientation.w])
        initial_pose = np.array([
                req.initial_pose.pose.position.x, req.initial_pose.pose.position.y, req.initial_pose.pose.position.z,
                rpy[0], rpy[1], rpy[2]])

        # Goal Pose
        rpy = tf.transformations.euler_from_quaternion([
                req.goal_pose.pose.orientation.x, req.goal_pose.pose.orientation.y,
                req.goal_pose.pose.orientation.z, req.goal_pose.pose.orientation.w])
        goal_pose = np.array([
                req.goal_pose.pose.position.x, req.goal_pose.pose.position.y, req.goal_pose.pose.position.z,
                rpy[0], rpy[1], rpy[2]])

        rospy.loginfo("Generating motion for dmp " + req.dmp_name)
        dmp = RollDmp(req.dmp_name, req.dt, "base_link")

        # Publish cartesian trajectory
        cartesian_trajectory, path = dmp.get_trajectory_and_path(goal_pose, initial_pose, req.tau)
        self.trajectory_pub.publish(cartesian_trajectory)
        self.path_pub.publish(path)

        # create and return service response
        response = GenerateMotionResponse()
        rospy.loginfo("Motion generated and published on respective toopics")
        response.result = "success"
        response.cart_traj = cartesian_trajectory
        return response


if __name__ == "__main__":
    obj = GenerateMotionClass()
    rospy.spin()
