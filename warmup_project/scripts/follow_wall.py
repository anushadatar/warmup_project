#!/usr/bin/env python3

import rospy
import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Twist, Vector3, Quaternion, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion, 
from visualization_msgs.msg import Marker

class Wall_Follower_Node(object):
    """ Node associated with wall following.  """
    def __init__(self):
        """ TODO """
        rospy.init_node('Wall_Follow')
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.analyzeScan)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.updatePosUsingOdom)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.r = rospy.Rate(10)
       
        # Parameters associated with the robot's position and scan view.
        self.wall_visible = False
        self.robot_position = Vector3(x=0, y=0, z=0) # Z axis is yaw
        self.next_move_msg = Twist(linear=Vector3(x=0), angular=Vector3(z=0))
        self.robot_direction = 1
        self.window = 45
        self.scan_view = []
        self.kp = 0.1
        self.speed = 0.1
        self.error = 0
        self.follow_distance = 1

        # Configure the wall visualization marker.
        self.marker = Marker()
        self.marker.action = Marker.ADD
        self.marker.ns = "vis_namespace"
        self.marker.id = 0
        self.marker.header.frame_id = "odom"
        self.marker.type = Marker.POINTS
        self.marker.scale = Vector3(0.1, 0.1, 0.1)
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0
        self.marker.color.a = 1.0

    def updatePosUsingOdom(self,msg):
        """ Use incoming odometry data to update the robot's position and yaw"""
        rotational_axes = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.robot_position.x = msg.pose.pose.position.x
        self.robot_position.y = msg.pose.pose.position.y
        self.robot_position.z = rotational_axes[2] #yaw

    def create_wall_marker(self):
        """ Find the position"""
        self.marker.header.stamp = rospy.Time.now()
        x = self.scan_view[0][0]*math.cos(self.scan_view[0][1])
        y = self.scan_view[0][0]*math.sin(self.scan_view[0][1])
        #add to current position
        self.marker.pose.position.x = self.robot_position.x+y
        self.marker.pose.position.y = self.robot_position.y-x
   
    def determine_turn_direction_and_speed(self):
        """ TODO """
        if self.scan_view[0][1] < 0.0: 
            self.robot_direction = 1
        else:
            self.robot_direction = -1

    def analyzeScan(self,msg):
        """ TODO """
        self.wall_visible = False
        self.scan_view = []

        total_range = list(range(0,self.window)) + list(range(360 - self.window, 360))
        for point in total_range:
            msg_distance = msg.ranges[point]
            if msg_distance == 0.0: 
                continue
            elif msg_distance < self.follow_distance*2:
                self.wall_visible = True
                # Adjust so the range is [-window, window]
                if point <= self.window:
                    self.scan_view.append([msg_distance, point*math.pi/180])
                else:
                    self.scan_view.append([msg_distance, (point-360)*math.pi/180])
          

        if self.wall_visible:
            # Sort the scan view so we can start at the closest point.
            self.scan_view.sort(key=lambda item: item[0])
            self.error = self.follow_distance - self.scan_view[0][0]
            self.create_wall_marker()
            self.determine_turn_direction_and_speed()

    def run(self):
        """TODO"""
        while not rospy.is_shutdown():
            if self.wall_visible:
                # TODO This needs to be a real proportional controller
                self.next_move_msg.angular.z = self.robot_direction*self.kp*self.error
            else:
                self.next_move_msg.linear.x = self.speed
                self.next_move_msg.angular.z = 0.0
            self.vel_pub.publish(self.next_move_msg)
            self.vis_pub.publish(self.marker)
            self.r.sleep()
            
if __name__ == '__main__':
	wall_follower_node = Wall_Follower_Node()
	wall_follower_node.run()
