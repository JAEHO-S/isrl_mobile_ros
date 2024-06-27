#!/usr/bin/env python
import rospy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from nav_msgs.msg import Odometry

def callback(rgb_image, depth_image, camera_info, scan, odom):
    # Publish synchronized messages to new topics
    pub_rgb.publish(rgb_image)
    pub_depth.publish(depth_image)
    pub_camera_info.publish(camera_info)
    pub_scan.publish(scan)
    pub_odom.publish(odom)

rospy.init_node('sync_node')

# Subscribers
rgb_image_sub = Subscriber('/zed_node/rgb/image_rect_color', Image)
depth_image_sub = Subscriber('/zed_node/depth/depth_registered', Image)
camera_info_sub = Subscriber('/zed_node/rgb/camera_info', CameraInfo)
scan_sub = Subscriber('/scan', LaserScan)
odom_sub = Subscriber('/odom', Odometry)

# Publishers
pub_rgb = rospy.Publisher('/sync_node/rgb/image_rect_color', Image, queue_size=30)
pub_depth = rospy.Publisher('/sync_node/depth/depth_registered', Image, queue_size=30)
pub_camera_info = rospy.Publisher('/sync_node/rgb/camera_info', CameraInfo, queue_size=30)
pub_scan = rospy.Publisher('/sync_node/scan', LaserScan, queue_size=30)
pub_odom = rospy.Publisher('/sync_node/odom', Odometry, queue_size=30)

# Approximate Time Synchronizer with adjusted slop and queue_size
ats = ApproximateTimeSynchronizer([rgb_image_sub, depth_image_sub, camera_info_sub, scan_sub, odom_sub], queue_size=100, slop=0.4)
ats.registerCallback(callback)

rospy.spin()
