#!/usr/bin/env python3

import rospy
import select
import sys
import termios
import tty

from geometry_msgs.msg import Twist, Vector3

class Teleop_Node():
    """
    Node associated with WASD-based neato teleoperation.
    """
    def __init__(self):
        """
        Kick off teleop node and associated cmd_vel publisher.
        """
        rospy.init_node('neato_teleop')
        self.settings = termios.tcgetattr(sys.stdin)
        self.key = None
        # Publish to cmd_vel based on the command input.
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # Movement commands based on Twist message type.
        self.forward = Twist(Vector3(1,0,0), Vector3(0,0,0))
        self.backward = Twist(Vector3(-1,0,0), Vector3(0,0,0))
        self.stop = Twist(Vector3(0,0,0), Vector3(0,0,0))
        self.left = Twist(Vector3(0,0,0), Vector3(0,0,1))
        self.right = Twist(Vector3(0,0,0), Vector3(0,0,-1))

    def run(self):
        """
        Process incoming command sequences from the keyboard.
        """
        while self.key != '\x03' and not rospy.is_shutdown():
            self.key = self.getKey()

            # Respond to key presses.
            if self.key == 'w':
                self.pub.publish(self.forward)
            elif self.key == 's':
                self.pub.publish(self.backward)
            elif self.key == 'a':
                self.pub.publish(self.left)
            elif self.key == 'd':
                self.pub.publish(self.right)
            else:
                self.pub.publish(self.stop)
        print('Program terminated.')


    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

if __name__ == "__main__":
    teleop = Teleop_Node()
    teleop.run()
