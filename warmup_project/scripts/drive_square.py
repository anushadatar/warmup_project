#!/usr/bin/env python3
import rospy
import math

from geometry_msgs.msg import Twist, Vector3
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

class Drive_Square_Odom_Node():
    """
    Use odometry data to drive the robot in a 1m x 1m square.
    """
    def __init__(self):
        """
        Initialize ROS and robot state parameters.
	    """
        # ROS Parameters.
        rospy.init_node("Drive_Square_Odom")
        self.r = rospy.Rate(10)	
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.updatePositionAndOrientation)

        # Robot State.
        self.go_straight_state = True
        self.robot_position = None # Type msg.pose.pose.positiom
        self.robot_orientation = None # Type msg.pose.pose.orientation
        self.side_length = 1 # Length for each side of the square
        self.speed = .2 # Default robot linear speed.
        self.initial_position = 0 # 
        self.initial_orientation = 0
        self.tolerance = 0.01
     
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
        Wait for odometry data, and then continuously drive in a square.
        """
        while not rospy.is_shutdown() and self.robot_position == None:
            print("Waiting for odometry data.")		
        self.reference_position = self.robot_position
        while not rospy.is_shutdown():
            if self.go_straight_state:
                self.driveStraight()
            else:
                self.driveRight()
        self.r.sleep()

if __name__ == '__main__':
	drive_square_odom_node = Drive_Square_Odom_Node()
	drive_square_odom_node.run()
