#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from nav_msgs.srv import GetPlan
import random
from nav_msgs.msg import OccupancyGrid

def is_location_available(startPose: Pose, moveBaseGoal: MoveBaseGoal):
    start = PoseStamped()
    start.header.seq = 0
    start.header.frame_id = "map"
    start.header.stamp = rospy.Time(0)
    start.pose = startPose

    goal = moveBaseGoal.target_pose

    get_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
    req = GetPlan()
    req.start = start
    req.goal = goal
    req.tolerance = 0
    resp = get_plan(req.start, req.goal, req.tolerance)
    #return resp
    if resp:
        return True
    else:
        return False

def select_rand_point_on_map(startPose: Pose, costmap : OccupancyGrid) -> MoveBaseGoal:
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    width_m = costmap.info.width * costmap.info.resolution
    height_m = costmap.info.height * costmap.info.resolution

    # Move ? meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = random.randint(-width_m, width_m)
    # Move ? meters forward along the y axis of the "map" coordinate frame 
    goal.target_pose.pose.position.y = random.randint(-height_m, height_m)
    # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

    while (not is_location_available(startPose, goal)):
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = random.randint(-width_m, width_m)
        goal.target_pose.pose.position.y = random.randint(-height_m, height_m)

    return goal

def get_test_goal():
    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()
    # Move ? meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = 0
    # Move ? meters forward along the y axis of the "map" coordinate frame 
    goal.target_pose.pose.position.y = 10
    # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = 1.0

    return goal

def move_to_goal(goal : MoveBaseGoal):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    wait = client.wait_for_result()
    # If the result doesn't arrive, assume the Server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

def rotate360():
    #Starts a new node
    rospy.init_node('map_explore', anonymous=True)
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()

    print("Rotating the robot by 360 deg")
    # larger than 360 because rospy.Time.now() has a slight delay
    angle = 490 #angle in deg
    speed = 40 #speed in deg/sec
    clockwise = True   

    #Converting from angles to radians
    relative_angle = np.deg2rad(angle)
    angular_speed = np.deg2rad(speed)

    #We wont use linear components
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0

    # Checking if our movement is CW or CCW
    if clockwise:
        vel_msg.angular.z = -abs(angular_speed)
    else:
        vel_msg.angular.z = abs(angular_speed)
    # Setting the current time for distance calculus
    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        velocity_publisher.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)


    #Forcing our robot to stop
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    rospy.spin()