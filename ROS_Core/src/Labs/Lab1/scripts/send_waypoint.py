#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion

def send_waypoint(x, y, z, frame_id="map", qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    rospy.init_node('send_waypoint_node', anonymous=True)
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(1)  # Sleep to give the publisher time to connect to subscribers

    # Create the PoseStamped message
    waypoint = PoseStamped()
    waypoint.header.frame_id = frame_id
    waypoint.header.stamp = rospy.Time.now()

    waypoint.pose.position.x = x
    waypoint.pose.position.y = y
    waypoint.pose.position.z = z
    waypoint.pose.orientation = Quaternion(qx, qy, qz, qw)

    # Publish the waypoint
    pub.publish(waypoint)
    rospy.loginfo("Waypoint sent: {}".format(waypoint))

if __name__ == '__main__':
    print("SEND WAYPOINT NODE STARTING UP!")
    # Example waypoint coordinates
    send_waypoint(1.0, 2.0, 0.0)