#!/usr/bin/env python3
import rospy
import time 

from geometry_msgs.msg import Twist, Vector3

class Drive_Square_Node():
    """
    TODO: Drive in a square using /odom as well.
    Node that drives the Neato in a square using timing.
    """
    def __init__(self):
        """
        Initialize ROS and directional parameters.
        """
        # ROS Parameters.
        rospy.init_node("Drive_Square")
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # Directional messages.
        self.neato_turn = Twist(Vector3(0, 0, 0), Vector3(0, 0, 1))
        self.neato_stop = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        self.neato_forward = Twist(Vector3(1, 0, 0), Vector3(0, 0, 0))
    
    def turn_neato(self):
        """
        Publish velocity messages to stop, move forward, and then turn the neato.
        """
        self.pub.publish(self.neato_stop)
        rospy.sleep(2)
        self.pub.publish(self.neato_forward)
        rospy.sleep(1)
        self.pub.publish(self.neato_turn)
        rospy.sleep(2)

    def run(self):
        """
        Turns the neato 4 times to execute on driving in a square.
        """
        for i in range(4):
            self.turn_neato()

if __name__ == "__main__":
    drive_square = Drive_Square_Node()
    drive_square.run()
