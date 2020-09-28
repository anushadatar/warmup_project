#!/usr/bin/env python3

import rospy
import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Twist, Vector3, Quaternion, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

class Wall_Follower_Node(object):
    """ 
    Use a proportional controller to visualize and follow a wall.
    Specifically, this node uses the LaserScan input to find the nearest wall,
    uses odometry data to create a marker to visualize the wall's location, and
    then uses proportional control to adjust the linear velocity and
    steering parameters of the robot so that it follows the wall.
    """
    def __init__(self):
        """
        Initialize ROS, proportional control, and wall marker parameters.        
        """
        # ROS Parameters.
        rospy.init_node('Wall_Follow')
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.analyzeScan)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.updatePosUsingOdom)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.r = rospy.Rate(10)
       
        # Controller parameters.
        self.follow_distance = 1 # Approx. distance away from the wall.
        self.speed = 0.1 # Default linear speed.
        self.wall_visible = False # Should default to false.
        self.robot_direction = 1 # Should default to 1.
        self.window = 45 # Scanning window for each side (in degrees).
        self.scan_view = [] # Holds processed LaserScan.
        self.kp = 0.2 # Proportional constant for linear speed.
        self.error = 0 # Error for controller, should default to 0
        self.next_move_msg = Twist(linear=Vector3(x=0), angular=Vector3(z=0))

        # Visualization parameters for wall marker.
        self.robot_position = Vector3(x=0, y=0, z=0) # Z axis is yaw
        self.marker = Marker()
        self.marker.action = Marker.ADD
        self.marker.ns = "vis_namespace"
        self.marker.id = 0
        self.marker.header.frame_id = "odom"
        self.marker.type = Marker.POINTS
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.1
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0
        self.marker.color.a = 1.0

    def updatePosUsingOdom(self,msg):
        """ 
        Use incoming odometry data to update the robot's position and yaw.

        Args:
        data: Incoming Odometry messages.

        Returns:
        Void, and updates self.robot_position with x, y, and yaw.
        """
        rotational_axes = euler_from_quaternion((msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))
        self.robot_position.x = msg.pose.pose.position.x
        self.robot_position.y = msg.pose.pose.position.y
        self.robot_position.z = rotational_axes[2] # Yaw

    def createWallMarker(self):
        """ 
        Create a marker for the detected wall so that it can be visualized
        in rviz. This just creates the marker; it still needs publishing.
        """
        self.marker.header.stamp = rospy.Time.now()
        # Convert scan view to cartesian points.
        x = self.scan_view[0][0]*math.cos(self.scan_view[0][1])
        y = self.scan_view[0][0]*math.sin(self.scan_view[0][1])
        # Add scanned points to robot position to get marker position.
        self.marker.pose.position.x = self.robot_position.x+y
        self.marker.pose.position.y = self.robot_position.y-x

    def analyzeScan(self,msg):
        """ 
        Analyze incoming LaserScan data to find a wall and calculate the
        distance the robot is from it.
        """
        self.wall_visible = False
        self.scan_view = []
    
        # Check both sides of the window.
        total_range = list(range(0,self.window)) + list(range(360 - self.window, 360))
        for point in total_range:
            msg_distance = msg.ranges[point]
            if msg_distance == 0.0 or msg_distance == float('inf') or msg_distance == float('-inf'):
                continue
            if msg_distance < self.follow_distance*2:    
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
            self.createWallMarker()

    def run(self):
        """
        Adjust direction and velocity to follow visible walls.
        """
        while not rospy.is_shutdown():
            if self.wall_visible:
                # Proportionally adjust based on scan view.
                if self.scan_view[0][1] < 0.0:
                    self.robot_direction = 1
                else:
                    self.robot_direction = -1
                self.next_move_msg.angular.z = self.robot_direction*self.kp*self.error
            else:
                self.next_move_msg.linear.x = self.speed
                self.next_move_msg.angular.z = 0.0
            # Publish the next move and visualization marker.
            self.vel_pub.publish(self.next_move_msg)
            self.vis_pub.publish(self.marker)
            self.r.sleep()
            
if __name__ == '__main__':
	wall_follower_node = Wall_Follower_Node()
	wall_follower_node.run()
