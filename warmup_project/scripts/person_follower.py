#!/usr/bin/env python3

import math
import numpy
import rospy

from geometry_msgs.msg import Twist, Vector3, Point, Quaternion
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

class Person_Follower_Node(object):
    """
    Use a proportional controller to visualize and follow a person.
    """
    def __init__(self):
        """
        Initialize ROS, proportional control, and person marker parameters.
        """
        rospy.init_node("Person_Follower")
        self.pub_vel = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
        self.pub_vis = rospy.Publisher("/vis_scan",Marker,queue_size=10)	
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.updateScanRanges)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.updatePositionAndOrientation)
        self.r = rospy.Rate(10)

        # Robot state.
        self.scan_ranges = None
        self.robot_position = Vector3(x=0, y=0) # Type msg.pose.pose.position.
        self.robot_orientation = None # Type msg.pose.pose.orientation.
        self.x_center = 0 # X coordinate of the center of mass.
        self.y_center = 0 # Y coordinate of the center of mass.
        self.next_move_msg = Twist(linear=Vector3(x=0), angular=Vector3(z=0))        
        self.kp_speed = 0.7 # Proportional constant for forward speed.
        self.kp_angle = 0.3 # Proportional constant for angle.
        self.speed = 1.4 # Default linear speed, cut by kp.
        self.error = 0 # Error for controller, should default to 0
        self.follow_distance = 0.05 # Distance from person to follow at.
        self.max_speed = 0.3

        # Marker parameters.
        self.person_marker = Marker()
        self.person_marker.header.frame_id = "odom"
        self.person_marker.action = Marker.ADD
        self.person_marker.ns = "vis_namespace"
        self.person_marker.id = 0
        self.person_marker.scale.x = 0.1
        self.person_marker.scale.y = 0.1
        self.person_marker.scale = Vector3(0.02, 0.02, 0.02)
        self.person_marker.type = Marker.POINTS
        self.person_marker.color.r = 1.0
        self.person_marker.color.g = 0.0
        self.person_marker.color.b = 1.0
        self.person_marker.color.a = 1.0
        
        # Distance formula function for calculating distance
        self.distance_formula = lambda initial, final : math.sqrt((initial.x-final.x)**2 + (initial.y-final.y)**2)

    def updateScanRanges(self, data):
        """ 
        Store the most recent laser scan data in the scan_ranges variable. 
        """
        self.scan_ranges = data.ranges


    def updatePositionAndOrientation(self,data):
        """
        Store incoming odometry data to self.robot_position and
        self.robot_orientation.
        """
        pose = data.pose.pose
        self.robot_position = pose.position
        self.robot_orientation = euler_from_quaternion([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])[0]

    def findPerson(self):
        """
        Compute the center of mass of the scan to find the person. 
        """
        # Compute the x and y coordinates of the center of mass.
        if not self.scan_ranges:
            return
        x_coords = []
        y_coords = []
        for angle in range(0, 360):
            if not math.isinf(self.scan_ranges[angle]):
                x_coords.append(self.scan_ranges[angle] * math.sin(angle))
                y_coords.append(self.scan_ranges[angle] * math.cos(angle))
        # If we see x and y coords, assign the average to the center.
        if len(x_coords) > 0:
            self.x_center = sum(x_coords)/len(x_coords)
        else:
            self.x_center = 0
        if len(y_coords) > 0:
            self.y_center = sum(y_coords)/len(y_coords)
        else:
            self.y_center = 0

    def approachPerson(self):
        """
        Approach the detected center of mass, if one has been detected.
        Otherwise, simply continue moving forward.
        """
        if self.distanceFromRobot() > self.follow_distance:
            if self.y_center != 0:
                # Turn if needed.
                self.next_move_msg.angular.z = self.kp_angle*math.tanh(self.x_center/self.y_center)
            else:
                # Otherwise go straight.
                self.next_move_msg.angular.z = 0
            self.next_move_msg.linear.x = self.kp_speed*self.speed*(self.distanceFromRobot() - self.follow_distance)
        else:
            # Stop if too close.
            self.next_move_msg.linear.x = 0
            self.next_move_msg.angular.z = 0

    def visCenterOfMass(self):
        """
        Create and publish a marker by which to visualize the center of
        mass of the points visible to the neato.
        """
        self.person_marker.header.stamp = rospy.Time.now()
        self.person_marker.pose.position.x = self.robot_position.x+self.x_center
        self.person_marker.pose.position.y = self.robot_position.y+self.y_center
        self.person_marker.points.append(Point(self.x_center, self.y_center, 1))
        self.pub_vis.publish(self.person_marker)

    def distanceFromRobot(self):
        """
        Returns the distance between the robot's position and the perceived
        center of mass.
        """
        return math.sqrt((self.x_center)**2 + (self.y_center)**2)

    def run(self):
        """
        Start by moving forward. From there, find the person, visualize the
        center of mass, approach the person, and repeat over time.
        """
        self.next_move_msg.linear.x = self.speed
        self.pub_vel.publish(self.next_move_msg)
        while not rospy.is_shutdown():
            self.findPerson()
            self.visCenterOfMass()
            self.approachPerson()
            if self.next_move_msg.linear.x > self.max_speed:
                self.next_move_msg.linear.x = self.max_speed
            self.pub_vel.publish(self.next_move_msg)
            self.r.sleep()

if __name__ == '__main__':
    person_follower_node = Person_Follower_Node()
    person_follower_node.run()
