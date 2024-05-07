#!/usr/bin/env python
import rospy
import yaml
from geometry_msgs.msg import PoseStamped
import numpy as np
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Path


car_x = 0
car_y = 0
obstacles = []
switchpoint = [0,0,0]
last_replan_time = None
last_switchpoint = [0,0,0]

def callback(data):
    global car_x
    global car_y
    for marker in data.markers:
        car_x = marker.pose.position.x
        car_y = marker.pose.position.y
    #print("x,y:", car_x,car_y)

def obstacle_callback(data):
    global obstacles
    arr = []
    for marker in data.markers:
        x = marker.pose.position.x
        y = marker.pose.position.y
        arr.append([x,y])
    obstacles = arr

def red_path_callback(data):
    pass
    #poses = data.poses

def switchpoint_callback(data):
    global switchpoint
    x = data.pose.position.x
    y = data.pose.position.y
    percent = data.pose.position.z = 0
    switchpoint = [x,y,percent]
    #print("sp: ", switchpoint)

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
    rospy.Subscriber("/Obstacles/Static", MarkerArray, obstacle_callback)
    rospy.Subscriber("/Routing/Path", Path, red_path_callback)
    rospy.Subscriber("/switchpoint", PoseStamped, switchpoint_callback)
    
    pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.init_node('waypoint_publisher', anonymous=True)
    

    task_1 = "../task1.yaml"
    data = load_waypoints(task_1)
    keys = []
    waypoints = [] #waypoints in order
    backup_wps = []
    names = []
    for x in data:
        keys.append(x)

    for i, val in enumerate(data['goal_order']):
        waypoints.append(data[keys[val-1]])
        backup_wps.append(data['backups'][val-1])
        names.append(val)

    start_time = rospy.Time.now().to_sec()

    #print("Waypoints in order: ", waypoints)
    #print("Backups to those waypoints in order: ", backup_wps)

    #rpath = util.RefPath()
    #print(rpath.center_line)
    
    #waypoints = waypoints[1:] # gets rid of first wp

    pose = PoseStamped()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "map"
    pose.pose.position.x = waypoints[0][0]
    pose.pose.position.y = waypoints[0][1]
    pose.pose.position.z = 0
    pose.pose.orientation.w = 1.0
    pub.publish(pose)

    for i in range(len(waypoints)):
        for obstacle in obstacles: #obstacles is updated from the callback function
            dist_wp_to_obs = np.linalg.norm(np.array([obstacle[0], obstacle[1]]) - np.array([waypoints[i][0], waypoints[i][1]]))
            if dist_wp_to_obs < 0.2:
                print("Object in way! Switching waypoint: ", waypoints[i], " to: ", backup_wps[i])
                waypoints[i] = backup_wps[i]


        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = waypoints[i][0]
        pose.pose.position.y = waypoints[i][1]
        pose.pose.position.z = 0
        pose.pose.orientation.w = 1.0
        pub.publish(pose)
        last_replan_time = rospy.Time.now().to_sec()
        rospy.loginfo("Published a new waypoint: (" + str(waypoints[i][0]) +  ", " + str(waypoints[i][1]) +  ") which is location: " + str(names[i]))
        distance_to_goal = np.linalg.norm(np.array([car_x, car_y]) - np.array([waypoints[i][0], waypoints[i][1]]))
        distance_to_switchpoint = np.linalg.norm(np.array([switchpoint[0], switchpoint[1]]) - np.array([waypoints[i][0], waypoints[i][1]]))

        while True:
            
            # fixes issue of skipping a waypoint... gives time for the ros topics to be retrieved
            while(last_switchpoint == switchpoint):
                rospy.sleep(0.05)
                if names[i] == 1:
                    rospy.sleep(0.1)
                    break
            
            #car and switchpoint locations are being updated by the callbacks
            distance_to_goal = np.linalg.norm(np.array([car_x, car_y]) - np.array([waypoints[i][0], waypoints[i][1]]))
            distance_to_switchpoint = np.linalg.norm(np.array([switchpoint[0], switchpoint[1]]) - np.array([car_x, car_y]))
            #print("     Distance to switch: ", distance_to_switchpoint)
            #print("     Distance to goal: ", distance_to_goal)
            #print()
            # to change the percentage down the line of the switchpoint, look in traj_planner.py ("if val >")
            if distance_to_switchpoint < 0.3 or distance_to_goal < 0.3:
                time_now = rospy.Time.now().to_sec()
                time_since = time_now - last_replan_time
                time_total = time_now - start_time
                print("     Switching point: ", [switchpoint[0], switchpoint[1]])
                print("     Goal:", [waypoints[i][0], waypoints[i][1]])
                print("     Time since last replan: " +  str(time_since))
                print("     Total time: " +  str(time_total))
                if time_total > 5 and time_since < 0.5:
                    rospy.logerr("Time since last replan is suspiciously small...")
                last_switchpoint = switchpoint
                break 



    

