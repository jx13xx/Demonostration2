#!/usr/bin/env python
"""
    my_node.py

    A ROS node that repeats the map and odometry topic to the correct ecte477 
    namespace topics for map and path.

    Subscribed: map/, odom/
    Publishes: ecte477/map/, ecte477/path/
    Services: explore/explore_service
    Created: 2021/04/08
    Author: Brendan Halloran
"""

import rospy
import time
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from std_srvs.srv import SetBool
from demonstration2.msg import custom
from datetime import datetime

class my_node:
    def __init__(self):
        # Subs and pubs
        self.subscriber_frontiers = rospy.Subscriber('/explore/frontiers', MarkerArray, self.callback_frontiers)
        self.subscriber_map = rospy.Subscriber('/map', OccupancyGrid, self.callback_map)
        self.subscriber_odom = rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.publisher_move_base_simple_goal = rospy.Publisher('move_base_simple/goal/', PoseStamped, queue_size=1)
        self.publisher_map = rospy.Publisher('/ecte477/map', OccupancyGrid, queue_size=1)
        self.publisher_path = rospy.Publisher('/ecte477/path', Path, queue_size=1)
        #load the beacon file
        self.beacons = rospy.get_param("beacons")
        #publish the beacon
        self.beaconsPub = rospy.Publisher('/ecte477/beacons',custom, queue_size = 10)
        msg = custom()
        # Object for storing path
        self.path = Path()
        self.path.header.frame_id = "odom"
        # Start explore_lite after 5 seconds
        self.rate = rospy.Rate(0.2)
        self.rate.sleep()
        rospy.wait_for_service('explore/explore_service')
        start_explore_lite = rospy.ServiceProxy('explore/explore_service', SetBool)
        resp  = start_explore_lite(True)
        test = "dfsfsdASDSAD"


        for x in self.beacons:
            id = x["id"]
            top_color = x["top"]
            bottom_color = x["bottom"]
            print("Beacon number {} is {} on the top and {} on the bottomr".format(id,top_color, bottom_color))
            #may-required/ may not we need to check

            custom.header.data = id + " " + rospy.Time.now()
            custom.header.frame_id = 'map'
            custom.top.data = top_color
            custom.bottom.data = bottom_color
            # add the marker value here for the position
            custom.position.data = "null"
            self.beaconsPub.publish(custom)
            if top_color == "blue" and bottom_color == "red":
                print("The blue-red beacon is found")
		
    # Simply repeact the map data to the correct topic
    def callback_map(self, data):
        self.publisher_map.publish(data);
        
    # Turn the odometry info into a path and repeat it to the correct topic
    def callback_odom(self, data):
        pose = PoseStamped()
        pose.pose = data.pose.pose
        self.path.poses.append(pose)
        self.publisher_path.publish(self.path)
        
    def callback_frontiers(self, frontiers):
        if len(frontiers.markers) == 0:
            rospy.loginfo("Returning Home")
            goal_msg = PoseStamped()
            goal_msg.header.stamp = rospy.Time.now()
            goal_msg.header.frame_id = 'map'
            goal_msg.pose.orientation.w = 1.0;
            self.publisher_move_base_simple_goal.publish(goal_msg)
	
	
	
# Main function
if __name__ == '__main__':
    rospy.init_node('my_node', anonymous=True)
    rospy.loginfo("Starting My Node!")
    mn = my_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down My Node!")
