#!/usr/bin/env python
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
import numpy as np
from visualization_msgs.msg import MarkerArray

car_x = 0
car_y = 0

def callback(data):
    global car_x
    global car_y
    for marker in data.markers:
        car_x = marker.pose.position.x
        car_y = marker.pose.position.y
    #print("x,y:", car_x,car_y)

def listener():
    rospy.init_node('truck_marker_subscriber', anonymous=True)
    rospy.Subscriber("/vis/truck", MarkerArray, callback)
    rospy.spin()

def load_waypoints(filename):
    with open(filename, 'r') as file:
        data = yaml.safe_load(file)
    return data



if __name__ == '__main__':

    rospy.Subscriber("/vis/truck", MarkerArray, callback)
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node('waypoint_publisher', anonymous=True)
    

    task_1 = "../task1.yaml"
    data = load_waypoints(task_1)
    keys = []
    waypoints = [] #waypoints in order
    for x in data:
        keys.append(x)
    for i, val in enumerate(data['goal_order']):
        waypoints.append(data[keys[val-1]])

    print("waypoints in order: ", waypoints)

    for i in range(len(waypoints)):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = waypoints[i][0]
        pose.pose.position.y = waypoints[i][1]
        pose.pose.position.z = 0
        pose.pose.orientation.w = 1.0
        pub.publish(pose)
        print("published")
        distance_to_goal = np.linalg.norm(np.array([car_x, car_y]) - np.array([waypoints[i][0], waypoints[i][1]]))
        while distance_to_goal > 1:
            rospy.sleep(0.04)
            distance_to_goal = np.linalg.norm(np.array([car_x, car_y]) - np.array([waypoints[i][0], waypoints[i][1]]))
            print("Distance to goal: ", distance_to_goal)



    

