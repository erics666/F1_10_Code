#!/usr/bin/env python

from __future__ import print_function
import rospy

# TODO: import ROS msg types and libraries
import sys
import numpy as np
import csv
from os.path import expanduser
from visualization_msgs.msg import Marker
from std_msgs.msg import String

def main():
    rospy.init_node('pure_pursuit_node')
    home = expanduser('~')
    filepath=home+'/rcws/logs/wp-test10.csv' 
    with open(filepath,'r') as csvfile:
        data = list(csv.reader(csvfile))
        waypoints_tmp=np.asarray(data)  #tansfer list to array
        waypoints=waypoints_tmp.astype(np.float32) #transfer string to float 
    
    marker_pub = rospy.Publisher('wp', Marker, queue_size=10)
    marker=Marker()
    marker_pub.publish(marker)

    speaker_pub = rospy.Publisher('chatter', String, queue_size=10)
    rate=rospy.Rate(10)

    #self.marker.action=3
    #self.marker_pub.publish(self.marker)
    while not rospy.is_shutdown():
        for i in range(len(waypoints)):
            marker.ns = "Waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.header.frame_id = 'map'
            marker.action = Marker.ADD
            marker.pose.position.x = waypoints[i,0]
            marker.pose.position.y = waypoints[i,1]
            marker.pose.position.z = 0.5
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0 #Don't forget to set the alpha!
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker_pub.publish(marker)
        rate.sleep()

        #rospy.loginfo(marker)
        #only if using a MESH_RESOURCE marker type:
        #self.marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae"
    rospy.spin()
        
    
    
if __name__ == '__main__':
    main()
