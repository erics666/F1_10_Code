#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback) 
	#TODO sub to lidar
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10) 
	#TODO pub to drive
    
    def preprocess_lidar(self, ranges, data):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
	1080 middle point is 0 degrees
	-pi, +pi
        """
	i = 0
        proc_ranges = list(ranges)

	indexValue1 = int((math.radians(90) - data.angle_min)/(data.angle_increment))  #like 812
	indexValue2 = int((math.radians(-90) - data.angle_min)/(data.angle_increment)) #like 270

	proc_ranges[0: indexValue2]= [0]*indexValue2  
	proc_ranges[indexValue1: len(proc_ranges)]= [0]*(len(proc_ranges)-indexValue1) 

	proc_ranges[proc_ranges > data.range_max] = data.range_max
	proc_ranges[proc_ranges < data.range_min] = 0

        return proc_ranges

    def find_max_gap(self, free_space_ranges):
        """ Return the start index & end index of the max gap in free_space_ranges
        """
	i = 0 # a variable to go through the array's indices
        gStart = -1 # to mark the start of a gap
	gEnd = -1 # to mark the end of a gap
	bigG = 0 # will hold the biggest non-zero gap's size
	bigGStart = 0 # will hold the biggest non-zero gap's starting index
	bigGEnd = 0 # will hold the biggest non-zero gap's ending index

	while i <= len(free_space_ranges)-1: # travel through the array
	    if (free_space_ranges[i] != 0) and (gStart == -1): 
		gStart = i
	    elif (free_space_ranges[i] == 0) and (gStart != -1):
		gEnd = i
		if (gEnd - gStart) > bigG:
		    bigG = gEnd - gStart
	    	    bigGStart = gStart
	    	    bigGEnd = gEnd
		gStart = -1
	    i = i + 1
	arr = [bigGStart, bigGEnd]
        return arr #return the start and end of the biggest gap
    
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        i = start_i
	bestPointIndex = 0
	bestPoint = 0
	while i <= end_i:
	    if ranges[i] > bestPoint:
		bestPoint = ranges[i]
		bestPointIndex = i
	    i = i + 1
	    #print(ranges[i])
	best = [bestPointIndex, bestPoint]

        return best


    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = list(data.ranges)
        #ranges = self.preprocess_lidar(preproc_ranges, data)
        #Find closest point to LiDAR
	

	minV = min(ranges)	# the closest value to the lidar

        #Eliminate all points inside 'bubble' (set them to zero) 

	minVIndex = ranges.index(minV)	# use minVIndex to find the index of the closest value in a not sorted array
	bub = -220	# a number to create the bubble size, it should half the total size
	ranges[minVIndex + bub: minVIndex-bub]=[0]*(-bub*2)  # set the bubble values to 0
        #print("index", minVIndex)
        print("value", minV)
        ranges = self.preprocess_lidar(ranges, data)#space behind the car


        #Find max length gap 
	arr = self.find_max_gap(ranges)
	biggestGapStart=arr[0]  
	biggestGapEnd=arr[1]
	biggestGap = biggestGapEnd - biggestGapStart # the size of the biggest gap
		
        #Find the best point in the gap 
	bestPoint = self.find_best_point(biggestGapStart, biggestGapEnd, ranges) 
	velocity=0.8
        #Publish Drive message
	drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
	indexValue = bestPoint[0]
	radAngle =  ((data.angle_increment)*(indexValue) + data.angle_min) 
        drive_msg.drive.steering_angle = radAngle 	# have not found the right angle yet
	# 0 is straight ahead, 90 degrees is left, -90 is right
	#int((data.angle_increment)*(indexValue) + data.angle_min)
	degAngle = (180*radAngle/math.pi)
       
	#print("angle", degAngle)
	#print(arr[0])
	# data.angle_increment = 0.0058, got from (3.14 - (-3.14))/1080
	# data.angle_min = -3.14 in simulator
	# indexValue should be 0 to 1080
        drive_msg.drive.speed = velocity	
        self.drive_pub.publish(drive_msg)

	


def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.1)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
