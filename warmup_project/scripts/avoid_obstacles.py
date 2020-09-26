#!/usr/bin/env python3
""" This script makes a robot avoid obstacles"""
import rospy
import math

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

class Avoid_Obstacles_Node(object):
    """ TODO """
    def __init__(self):
        """ TODO """
        rospy.init_node('Avoid_Obstacles')
        self.r = rospy.Rate(10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.analyzeScan)
       # self.odom_sub = rospy.Subscriber('/odom', Odometry, self.updatePosUsingOdom)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.robot_position = Vector3(x=0, y=0, z=0) # Z axis is angle
        self.speed = 0.2
        self.next_move_msg = Twist(linear=Vector3(x=self.speed), angular=Vector3(z=0))
        # TODO this needs some pretty serious tuning
        self.kp_distance = 0.1
        self.kp_angle = 0.7
        self.window = 90
        
        # Potential fields
        self.desired_distance = 1
        self.threshold = 3
        self.desired_angle = 0
        self.potential_spread = 1
        self.potential_radius = 0
        self.potential_total = self.potential_spread + self.potential_radius
        self.potential_offset = 0.1

    def analyzeScan(self, data):
        """ TODO """ 
        total_x_value = 0
        total_y_value = 0
       
        for i in range(0, 360):
            if (data.ranges[i] < self.threshold) and (data.ranges[i] != 0.0):
                x = data.ranges[i]*math.cos(math.radians(i))
                y = data.ranges[i]*math.sin(math.radians(i))
                distance = math.sqrt(math.pow(x,2) + math.pow(y,2))
                difference = self.potential_total - distance
                if difference > 0:
                    angle = math.atan2(y, x)
                    total_x_value += -self.potential_offset*difference*math.cos(angle)
                    total_y_value += -self.potential_offset*difference*math.sin(angle)
        total_x_value += self.desired_distance
        self.next_move_msg.linear.x = math.sqrt(math.pow(total_x_value,2) + math.pow(total_y_value,2))*self.kp_distance
        self.next_move_msg.angular.z = math.atan2(total_y_value,total_x_value)*self.kp_angle

    def run(self):
        """ TODO """
        while not rospy.is_shutdown():
            # Literally just publish the velocity.
            self.vel_pub.publish(self.next_move_msg)
            # TODO Add visualization
            self.r.sleep()


if __name__ == '__main__':
    avoid_obstacles = Avoid_Obstacles_Node()
    avoid_obstacles.run()
