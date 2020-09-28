#!/usr/bin/env python3

import rospy
import math

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose, Twist, Vector3, Quaternion, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker

class Finite_State_Controller_Node(object):
    """ 
    Finite state controller that follows a wall when one is visible, otherwise
    drive in a 1 meter by 1 meter square (using odometry data to determine the
    coordinates of the square).
    """
    def __init__(self):
        """
        Initialize ROS, proportional control, wall marker, and square driving.        
        """
        # ROS Parameters.
        rospy.init_node('Finite_State_Controller')
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.analyzeScan)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.updatePositionAndOrientation)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.r = rospy.Rate(2)
       
        # Wall following controller parameters.
        self.follow_distance = 1 # Approx. distance away from the wall.
        self.speed = 0.1 # Default linear speed.
        self.wall_visible = False # Should default to false.
        self.robot_direction = 1 # Should default to 1.
        self.window = 45 # Scanning window for each side (in degrees).
        self.scan_view = [] # Holds processed LaserScan.
        self.kp = 0.2 # Proportional constant for linear speed.
        self.error = 0 # Error for controller, should default to 0.
        self.next_move_msg = Twist(linear=Vector3(x=0), angular=Vector3(z=0))

        # Visualization parameters for wall marker.
        self.robot_position = Vector3(x=0, y=0, z=0) # Z axis is yaw
        self.robot_orientation = None # Type msg.pose.pose.orientation
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
       
        # Square driving variables.
        self.go_straight_state = True # True = straight, False = right.
        self.side_length = 1 # Length for each side of the square.
        self.tolerance = 0.01 # Allowed measurement error.
     
        # Distance formula function for calculating progress.
        self.distance_formula = lambda initial, final : math.sqrt((initial.x-final.x)**2 + (initial.y-final.y)**2 + (initial.z-final.z)**2)

    def updatePositionAndOrientation(self,data):
        """
        Store incoming odometry data to self.robot_position and
        self.robot_orientation.
        """
        pose = data.pose.pose
        self.robot_position = pose.position
        self.robot_orientation = euler_from_quaternion([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])[0]

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
            
    def driveStraight(self):
        """
        Drive the robot straight until it meets or exceeds self.side_length.
        Then stop and change the state so that it will turn right.
        """
        if self.distance_formula(self.robot_position, self.reference_position) > self.side_length:
            # Switch to the turn state.
            self.reference_orientation = self.robot_orientation
            self.go_straight_state = False
            # Stop moving.
            self.vel_pub.publish(Twist())
        else:
            # Continue moving forward.
            self.vel_pub.publish(Twist(linear=Vector3(x=self.speed)))

    def driveRight(self):
        """
        Turn the robot to the right until it has turned 90 degrees.
        Then stop and change state so that it will drive straight.
        """
        if self.robot_orientation > self.reference_orientation + self.tolerance:
            self.reference_orientation += 2*math.pi # Flip frame if necessary
        if self.robot_orientation + math.pi/2 <= self.reference_orientation:
            self.go_straight_state = True
            self.vel_pub.publish(Twist())
            self.reference_position = self.robot_position
        else:
            self.vel_pub.publish(Twist(angular=Vector3(z=0.3)))

    def run(self):
        """
        Adjust direction and velocity to follow visible walls. If there is no
        visible wall, drive the robot in a square.
        """
        while not rospy.is_shutdown() and self.robot_position == None:
            print("Waiting for odometry data.")		
        # Start off moving forward.
        self.vel_pub.publish(Twist(linear=Vector3(x=self.speed)))
        # Only update the reference position when starting the square drive.
        started_square_drive = False
        self.reference_position = self.robot_position   
        while not rospy.is_shutdown():
            if self.wall_visible:
                started_square_drive = False
                # Proportionally adjust based on scan view.
                if self.scan_view[0][1] < 0.0:
                    self.robot_direction = 1
                else:
                    self.robot_direction = -1
                self.next_move_msg.linear.x = self.speed
                self.next_move_msg.angular.z = self.robot_direction*self.kp*self.error
                self.vel_pub.publish(self.next_move_msg)
                self.vis_pub.publish(self.marker)
                print("wall")
            else:
                # Drive in a square. Methods handle publishing.
                if not started_square_drive:
                    self.reference_position = self.robot_position
                    started_square_drive = True
                if self.go_straight_state:
                    self.driveStraight()
                else:
                    self.driveRight()
            self.r.sleep()
            
if __name__ == '__main__':
	finite_state_controller = Finite_State_Controller_Node()
	finite_state_controller.run()
