#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseWithCovarianceStamped

class StaticObstaclePublisher:
    def __init__(self):
        self.obstacles = []
        self.marker_array_publisher = rospy.Publisher('/Obstacles/Static', MarkerArray, queue_size=1)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.pose_estimate_callback)
        self.publish_timer = rospy.Timer(rospy.Duration(0.04), self.publish_obstacles)  # Publish every second
        rospy.loginfo("StaticObstaclePublisher initialized.")

    def pose_estimate_callback(self, msg):
        # Create a new marker for the estimated pose
        new_marker = Marker()
        new_marker.header.frame_id = msg.header.frame_id
        new_marker.header.stamp = rospy.Time.now()
        new_marker.ns = "static_obstacles"
        new_marker.id = len(self.obstacles)
        new_marker.type = Marker.CUBE
        new_marker.action = Marker.ADD
        new_marker.pose = msg.pose.pose
        new_marker.scale.x = 0.2
        new_marker.scale.y = 0.2
        new_marker.scale.z = 0.2
        new_marker.color.r = 0.0
        new_marker.color.g = 1.0
        new_marker.color.b = 0.0
        new_marker.color.a = 1.0

        # Add the new marker to the list of obstacles
        self.obstacles.append(new_marker)

    def publish_obstacles(self, event=None):
        marker_array = MarkerArray()
        marker_array.markers = self.obstacles
        self.marker_array_publisher.publish(marker_array)

if __name__ == '__main__':
    rospy.init_node('pose_obstacle_publisher')
    publisher = StaticObstaclePublisher()
    rospy.spin()
