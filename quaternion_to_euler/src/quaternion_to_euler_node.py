#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf.transformations
import math

class DataConverter:
    def __init__(self):
        # Create subscribers
        self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        self.amcl_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)
        
        # Store the latest data
        self.latest_imu_data = None
        self.latest_amcl_data = None
        
        # Create a timer to process data at regular intervals (e.g., every 1 second)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback)

    def imu_callback(self, data):
        self.latest_imu_data = data

    def amcl_callback(self, data):
        self.latest_amcl_data = data

    def timer_callback(self, event):
        if self.latest_imu_data:
            self.process_imu_data(self.latest_imu_data)

        if self.latest_amcl_data:
            self.process_amcl_data(self.latest_amcl_data)

    def process_imu_data(self, data):
        quaternion = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        # Convert radians to degrees
        euler_degrees = [math.degrees(angle) for angle in euler]
        rospy.loginfo("IMU - Roll: %f°, Pitch: %f°, Yaw: %f°" % (euler_degrees[0], euler_degrees[1], euler_degrees[2]))

    def process_amcl_data(self, data):
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        # Convert radians to degrees
        euler_degrees = [math.degrees(angle) for angle in euler]
        rospy.loginfo("AMCL - Roll: %f°, Pitch: %f°, Yaw: %f°" % (euler_degrees[0], euler_degrees[1], euler_degrees[2]))


def main():
    rospy.init_node('data_to_euler', anonymous=True)
    converter = DataConverter()
    rospy.spin()

if __name__ == '__main__':
    main()
