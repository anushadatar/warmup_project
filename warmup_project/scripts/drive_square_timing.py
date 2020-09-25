#!/usr/bin/env python3
import rospy
import time 

from geometry_msgs.msg import Twist, Vector3


class Drive_Square_Node():
    """
    Node associated with driving the Neato in a square.
    """
    def __init__(self):
        """
        Kick off square driving node and assocaited cmd_vel publisher.
        """
        rospy.init_node("Drive_Square")
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        
        # Define directional messages.
        self.neato_turn = Twist(Vector3(0, 0, 0), Vector3(0, 0, 1))
        self.neato_stop = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        self.neato_forward = Twist(Vector3(1, 0, 0), Vector3(0, 0, 0))
    
    def turn_neato(self):
        """
        Stop, move forward, and turn the neato.
        """
        self.pub.publish(self.neato_stop)
        rospy.sleep(2)
        self.pub.publish(self.neato_forward)
        rospy.sleep(2)
        self.pub.publish(self.neato_turn)
        rospy.sleep(2)

    def run(self):
        for i in range(4):
            self.turn_neato()

if __name__ == "__main__":
    drive_square = Drive_Square_Node()
    drive_square.run()
