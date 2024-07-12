#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped



def odometry_callback(odom_msg):
    # Create a PoseWithCovarianceStamped message
    pose_cov_msg = PoseWithCovarianceStamped()

    # Fill in the header information
    pose_cov_msg.header = odom_msg.header

    # Copy the pose information from Odometry message to PoseWithCovarianceStamped message
    pose_cov_msg.pose.pose = odom_msg.pose.pose

    # Publish the PoseWithCovarianceStamped message
    pose_cov_pub.publish(pose_cov_msg)



if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('odometry_to_pose_covariance')

    # Subscribe to the odometry topic
    rospy.Subscriber('/run_slam/camera_pose', Odometry, odometry_callback)

    # Create a publisher for the PoseWithCovarianceStamped message
    pose_cov_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    # Spin the node
    rospy.spin()
