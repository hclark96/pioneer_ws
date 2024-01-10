#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import  Quaternion

from sensor_msgs.msg import NavSatFix
import math
import numpy
import time
import serial


global lat1, lon1, lat2, lon2, quat, robotStatus, targetStatus 
f1 = 0
path_id = 0

rover_heading = 0.0
ref_heading = 10.00
heading_error_i = 0.0

gps1lat = 0.0
gps1lon = 0.0
gps2lat = 0.0
gps2lon = 0.0
rover_lat = 0.0
rover_lon = 0.0

ref_coord_1_lat = 0.1
ref_coord_1_lon = 0.12
ref_coord_2_lat = 0.13
ref_coord_2_lon = 0.14

history = []

#latitudes_field   = [37.260939600, 37.260467900]
#longitudes_field = [-121.839533600, -121.839519100]

latitudes_field   = [ref_coord_1_lat, ref_coord_2_lat]
longitudes_field = [ref_coord_1_lon, ref_coord_2_lon]

class getGPS(Node):

    def __init__(self):
        super().__init__('gps_subscriber')
        self.subscription = self.create_subscription(
		NavSatFix,
		'robot1/gps1', 
		1)
        self.subscription

    def listener_callback(self, msg):
        robotStatus = msg.status.status
        lat1 = msg.latitude
        lon1 = msg.longitude

class getQuat(Node):

    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
		Quaternion,
		'imu_quat',
		10)
        self.subscriptionC = self.create_subscription(
		Int16MultiArray,
		'imu/calibrationStatus',
		4)
        self.subscription

    def listener_callback(self, msg):
        quat = msg.quat
        yawRover = atan2(2.0*(quat[1]*quat[2] + quat[3]*quat[0]), quat[3]*quat[3] - quat[0]*quat[0] - quat[1]*quat[1] + quat[2]*quat[2]);
        
class getTarget(Node):

    def __init__(self):
        super().__init__('target_subscriber')
        self.subscription = self.create_subscription(
		NavSatFix,
		'robot1/target',
		1)
        self.subscription

    def listener_callback(self, msg):
        targetStatus = msg.status.status
        lat2 = msg.latitude
        lon2 = msg.longitude

class giveDirections(Node):

    def __init__(self):
        super().__init__('directions_publisher')
        self.publisher_ = self.create_publisher(
        	Twist,
        	'cmd_vel',
        	self.move_cmd_callback, 
        	5)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        bearingX = cos(lat2) * sin(lon2-lon1)
        bearingY = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2-lon1)
        yawTarget = atan2(bearingX,bearingY)
        yawTarget = 3.14/2
        yawDelta = yawTarget - yawRover
        
        dist = sqrt((lat2-lat1)**2 + (lon2-lon1)**2)
        
        msg = Twist()
        
        if dist > 0:
            msg.linear.x = 0.7
            msg.angular.z = 0.5 * degrees(yawDelta)/360
        
        #print(msg)
        self.publisher_.publish(msg)
        
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    getQuat = getQuat()
    getGPS = getGPS()
    getTarget = getTarget()
    giveDirections = giveDirections()

    rclpy.spin(getQuat,getGPS,getTarget,giveDirections)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    quat_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
