#!usr/bin/env python3

import rospy
import numpy as np

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

class SlamScaleFixer:
    def __init__(self):
        '''
        Constructor for the SlamScaleFixer class
        that corrects the z-axis of the t265 slam odometry
        using the rangefinder measurements
        '''
        # Initialize the node
        rospy.init_node('slam_scale_fixer')

        # initiliaze the scale factor for the z-axis with 0.7 (based on last tests on C2 cage)
        self.z_scale = rospy.get_param('intitial_z_scale', 0.7)
        # initialize the altitude that will stop to calculate the scale factor (one meter above the ground by def)
        self.max_corr_alt = rospy.get_param('max_corr_alt', 1)
        self.max_scale = rospy.get_param('max_scale', 2)
        self.min_scale = rospy.get_param('min_scale', 0.3)

        # Initialize the odom and range messages
        self.odom_msg = Odometry()
        self.range_msg = Range()

        # Initialize the publishers
        # publisher for the corrected odometry
        self.pub = rospy.Publisher('/camera/odom/sample_scaled', Odometry, queue_size=10)

        # Initialize the subscribers
        # t265 slam odometry subscriber
        rospy.Subscriber('/camera/odom/sample', Odometry, self.t265_odom_callback)
        # subscriber for the rangefinder
        rospy.Subscriber('/rangefinder/range', Range, self.rangefinder_callback)

        # dummy run
        self.run()

    def t265_odom_callback(self, msg):
        '''
        Callback function for the t265 slam odometry
        that receives the odometry message and corrects the z-axis
        and publishes the corrected odometry with the minimum latency possible
        '''
        self.odom_msg = msg

        # Correct the z-axis of the slam odometry
        self.odom_msg.pose.pose.position.z = self.odom_msg.pose.pose.position.z * self.z_scale

        # Publish the corrected odometry
        self.pub.publish(self.odom_msg)

    def rangefinder_callback(self, msg):
        '''
        Callback function for the rangefinder
        that receives the range message and calculates the scale factor
        '''
        self.range_msg = msg

        # check if the altitude is above the maximum altitude to calculate the scale factor
        if self.odom_msg.pose.pose.position.z > self.max_corr_alt:
            rospy.loginfo_once(f"Finished to calculate the scale factor. Reached the maximum altitude of {self.max_corr_alt} meters")
            rospy.loginfo_once(f"Scale factor for the z-axis: {self.z_scale}")
            return

        # Calculate the scale factor for the z-axis
        if self.odom_msg.pose.pose.position.z == 0 or self.range_msg.range == 0:
            return

        scale_tmp = self.range_msg.range / self.odom_msg.pose.pose.position.z

        # check if the scale factor is not zero
        if scale_tmp == 0:
            return

        # check if the scale factor is not too big or too small
        if scale_tmp > self.max_scale:
            self.z_scale = self.max_scale
            rospy.logdebug_throttle(1, f"Scale factor for the z-axis is too big. Setting it to the maximum value of {self.max_scale}")
        elif scale_tmp < self.min_scale:
            self.z_scale = self.min_scale
            rospy.logdebug_throttle(1, f"Scale factor for the z-axis is too small. Setting it to the minimum value of {self.min_scale}")
        else:
            self.z_scale = scale_tmp

        rospy.logdebug_throttle(1, f"Scale factor for the z-axis: {self.z_scale}")

    def run(self):

        # Set the rate of the node
        rate = rospy.Rate(10)

        # Keep the node running
        while not rospy.is_shutdown():
            rate.sleep()