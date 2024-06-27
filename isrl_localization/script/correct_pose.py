#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
# from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from robot_localization.srv import SetPose

amcl_pose = PoseWithCovarianceStamped()
reinit_pose = PoseWithCovarianceStamped()

def amcl_callback(msg):
    global amcl_pose
    amcl_pose = msg
    # print(get_pose)

def reinit_pose_callback(msg):
    global reinit_pose
    global amcl_pose
    print(amcl_pose.pose)
    reinit_pose.header.frame_id = "map"
    reinit_pose.pose.pose = amcl_pose.pose.pose
    reinit_pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.06854]
    pub.publish(reinit_pose)

rospy.init_node('pose_corrector', anonymous=True)

sub = rospy.Subscriber('amcl_pose', PoseWithCovarianceStamped,amcl_callback)
sub = rospy.Subscriber('correct_pose', Empty,reinit_pose_callback)
pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)

rospy.spin()
