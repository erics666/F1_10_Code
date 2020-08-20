#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 0.7 #TODO start with 8,0,0 and adjust bc this is unstable bc it doesnt account for future position
kd = 0.1 #TODO
ki = 0 #TODO 
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.55 #0.55
VELOCITY = 20.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters


class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback) #TODO: #Subscribe to LIDAR
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10) #TODO: #Publish to drive

    def getRange(self, data, angle):
        # Finish first, probably easiest
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right, lidar scan angles for a or/and b
        # Outputs length in meters to object with angle in lidar scan field of view, range at a or/and b,
        # make sure to take care of nans etc.
        #TODO: implement
        # data is scan topic data with 1080 data, angle is some angle into the wall, remember we used data ranges previously
        # use angle min and max from data to calculate the index, first use 90 degree angle, second is any angle you want,
        # maybe 70 or 60, data.ranges is the list of distances from 1080 data points from angle_min = -135 deg to angle_max = 135
        # data.ranges[0] = distance at -135 degrees
        # data.ranges[1080] = distance at 135 degrees
        # (75+135)/270 = x/1080 should give us index from 0 to 1080 of 90 degrees
        # (90+135)/270 = x/1080 should give us index at 90 degrees
        # (angle+135)/270 = x/1080 should give us the index of 'angle' degrees
        # put in ranges[index] should give us the distance to the wall at that angle

    	#b_index = int((-1.5 - data.angle_min) / data.angle_increment)
        #a_index = int(b_index + (angle/data.angle_increment)
        #a = data.ranges[a_index]
        #b = data.ranges[b_index]
        #return a, b;

        indexValue = int((angle - data.angle_min)/(data.angle_increment))

        # we use this eqn because the angle must adjust for the fact the angle_min is negative
        # and we want to correlate that to the index which is not negative
        # then we use the angle increment to scale which is the (anglemax-anglemin)/totalrange
        wallDist = data.ranges[indexValue]
        return wallDist

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        global VELOCITY
        #TODO: Use kp, ki & kd to implement a PID controller for 

        integral = integral + error
        diff = error - prev_error
        angle = kp * error + ki*integral + kd*diff
        prev_error = error
        
        if angle > .35: # 20 degreesact
            VELOCITY = 1.5
        elif angle > .175:
            VELOCITY = 2
        else:
	    VELOCITY = 2.5	
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)
        
        #print(angle) 
        


    def followLeft(self, data, leftDist):
        # Finish 2nd,
        #Follow left wall as per the algorithm 
        #TODO:implement, use the data to calculate error = futuredist - leftdist
        # use lidar to look left 2 distances a and b and calculate angle between
        # follow left should return error, so it can be used by lidar_callback
        # should be calling getRange twice to calculate distance, error, and angle
        
        angle_b = (math.pi)*(90)/(180) # change 90 and 60 degrees to radians
        angle_a = (math.pi)*(60)/(180)
        theta = angle_b - angle_a   # the difference btwn our arbitrary angles a and b
        b = self.getRange(data, angle_b) # length of car to wall at angle_b
        a = self.getRange(data, angle_a) # length of car to wall at angle_a
        alpha = math.atan( (b-a*math.cos(theta)) / (a*math.sin(theta)) ) # this is the offset angle, the angle
                                                                # between the x axis and the line parallel to AB
        dT = b * math.cos(alpha)    # the distance to wall at time T
        dT1 = dT - (VELOCITY * 0.35 * math.sin(alpha)) # the distance to wall at time T+1
        
        return dT1 - DESIRED_DISTANCE_LEFT   # the error, calculated by how much farther the car will be from the wall
                                    # in an additional measurement of time
            
    def lidar_callback(self, data):
        """ 
        """
        error = self.followLeft(data, DESIRED_DISTANCE_LEFT) #TODO: replace with error returned by #followLeft, this should return the error from followleft
        #send error to pid_control
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
        main(sys.argv)
