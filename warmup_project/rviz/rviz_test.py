#!/usr/bin/env python3

from visualization_msgs.msg import Marker
import rospy

class RvizSphereNode(object):
    """ Publishes a sphere in rviz """
    def __init__(self):
        rospy.init_node('test_rviz_sphere')
        self.pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    def run(self):
        r = rospy.Rate(10)
        marker = Marker();
        marker.header.frame_id = "odom";
        marker.header.stamp = rospy.Time.now();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = Marker.SPHERE;
        marker.action = Marker.ADD;
        marker.pose.position.x = 1;
        marker.pose.position.y = 2;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 1.0;
        marker.pose.orientation.y = 2.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.4;
        marker.scale.y = 0.4;
        marker.scale.z = 0.4;
        marker.color.a = 1.0; 
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        while not rospy.is_shutdown():
            marker.header.stamp = rospy.Time.now()
            self.pub.publish(marker);
            r.sleep()
    
if __name__ == '__main__':
    node = RvizSphereNode()
    node.run()
