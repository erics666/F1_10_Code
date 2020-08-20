#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from numpy import linalg as la
import os
import sys
import csv
import math
import tf
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion



# TODO: import ROS msg types and libraries
pointIndex = 0
path_points=[] 

dirname  = os.path.dirname(__file__)
filename = os.path.join(dirname, 'waypoint/wp-test12.csv')
with open(filename) as csvfile:
    data = list(csv.reader(csvfile))
    path_points_tmp=np.asarray(data)  
    path_points=path_points_tmp.astype(np.float32) #open and read the csv file

class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        # TODO: create ROS subscribers and publishers
        self.pose_msg = rospy.Subscriber('/odom', Odometry, self.pose_callback)
        self.drive_pub = rospy.Publisher('/nav', AckermannDriveStamped, queue_size=10)


    def pose_callback(self, pose_msg):
        # Import .csv file into a list (path_points)
        global pointIndex
        global path_points
       
        if pointIndex == len(path_points):
                pointIndex = 0
        x = pose_msg.pose.pose.position.x
        y = pose_msg.pose.pose.position.y
        
        #distance between robot and waypoint
        distance = math.sqrt(math.pow((path_points[pointIndex,0]-x),2)+math.pow((path_points[pointIndex,1]-y),2))
        if distance >= 3:        
        #select closest waypoint over 1.5 distance away
            qx=pose_msg.pose.pose.orientation.x
	    qy=pose_msg.pose.pose.orientation.y
	    qz=pose_msg.pose.pose.orientation.z
	    qw=pose_msg.pose.pose.orientation.w

	    quaternion = (qx,qy,qz,qw)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            yaw = euler[2]
            v1 = np.array([np.cos(yaw),np.sin(yaw)])
            v2 = np.array([path_points[pointIndex,0]-x, path_points[pointIndex,1]-y])
            angle = self.find_angle(v1, v2) #find angle of waypoint
            
            #TODO: calculate curvature/steering angle
            #if (angle > 0.4189) or (angle < -0.4189):
            #   angle = (angle/abs(angle))*0.4189 

            # TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
            drive_msg = AckermannDriveStamped()
            drive_msg.header.stamp = rospy.Time.now()
            drive_msg.header.frame_id = "laser"
	    drive_msg.drive.speed = 5
	    if (angle > 0.4189) or (angle < -0.4189):
               angle = (angle/abs(angle))*0.4189 
               drive_msg.drive.speed = 3
            drive_msg.drive.steering_angle = angle
            self.drive_pub.publish(drive_msg)
        else:   # if robot is too close to waypoint
            pointIndex = pointIndex + 1 # select next waypoint
            
    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
         v1 = v1/np.linalg.norm(v1)
         v2 = v2/np.linalg.norm(v2)
         v12 = np.cross(v1,v2)
         return math.asin(v12)


def main():
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    rospy.spin()
if __name__ == '__main__':
    main()
