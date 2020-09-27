#!/usr/bin/env python3
import rospy
import math

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker

class Avoid_Obstacles_Node(object):
    """
    Use potential fields to move forward while avoiding obstacles.
    Specifically, this node uses the LaserScan input to compute the total
    sum of repelling forces associated with objects in the path of the robot
    and then uses proportional control to adjust the linear velocity and
    steering parameters of the robot.
    """
    def __init__(self):
        """ 
        Initialize ROS, proportional control, and potential fields parameters.
        """
        # ROS Parameters.
        rospy.init_node('Avoid_Obstacles')
        self.r = rospy.Rate(10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.analyzeScan)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Controller Parameters.
        self.speed = 0.2
        self.next_move_msg = Twist(linear=Vector3(x=self.speed), angular=Vector3(z=0))
        self.kp_distance = 0.3 # Proportional constant for linear speed.
        self.kp_angle = 0.5 # Proportional constant for steering.
        
        # Potential Fields Parameters.
        self.force_threshold = 1 # Cutoff for pos vs. neg force.
        self.potential_offset = 0.1 # Weight for each object.

    def analyzeScan(self, data):
        """
        Use potential fields to characterize obstacles and update the robot's
        next move to avoid the obstacles and continue moving forward. Acts as
        the callback function for the LaserScan subscriber.

        Args:
        data: Incoming LaserScan messages.

        Returns:
        Void. Updates self.next_move_msg with new values based on scan.
        """ 
        # Cumulative sums of forces.
        total_x_value= 0
        total_y_value = 0
       
        for scan_angle, value in enumerate(data.ranges):
            if (value < self.force_threshold) and (value != 0.0):
                x = value*math.cos(math.radians(scan_angle))
                y = value*math.sin(math.radians(scan_angle))
                distance = math.sqrt(math.pow(x,2) + math.pow(y,2))
                # Adjust angle if necessary.
                if distance < 1:
                    robot_angle = math.atan2(y, x)
                    # Get "other side" of distance by subtracting from 1.
                    total_x_value += -self.potential_offset*(1 - distance)*math.cos(robot_angle)
                    total_y_value += -self.potential_offset*(1 - distance)*math.sin(robot_angle)
        # Positive force for foward motion.
        total_x_value += self.force_threshold
        
        # Find next move messages, offset by proportional constant.
        self.next_move_msg.linear.x = math.sqrt(math.pow(total_x_value,2) + math.pow(total_y_value,2))*self.kp_distance
        self.next_move_msg.angular.z = math.atan2(total_y_value,total_x_value)*self.kp_angle

    def run(self):
        """ 
        Run the potential fields obstacle avoidance algorithm by publishing the
        next_move_msg which is constantly updated by the callback function
        Avoid_Obstacles_Node.analyzeScan.
        """
        while not rospy.is_shutdown():
            self.vel_pub.publish(self.next_move_msg)
            self.r.sleep()


if __name__ == '__main__':
    avoid_obstacles = Avoid_Obstacles_Node()
    avoid_obstacles.run()
