#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose, PoseWithCovarianceStamped
import numpy as np
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from nav_msgs.srv import GetPlan
import random
from nav_msgs.msg import OccupancyGrid
from dr_phil_hardware.srv import GenerateTarget





def is_location_available(startPose: Pose, moveBaseGoal: MoveBaseGoal):
    start = PoseStamped()
    start.header.seq = 0
    start.header.frame_id = "map"
    start.header.stamp = rospy.Time(0)
    start.pose = startPose

    goal = moveBaseGoal.target_pose
    rospy.wait_for_service('/move_base/make_plan')
    get_plan = rospy.ServiceProxy('/move_base/make_plan', GetPlan)
    req = GetPlan()
    req.start = start
    req.goal = goal
    req.tolerance = 0.1
    resp = get_plan(req.start, req.goal, req.tolerance)
    #return resp
    if resp:
        return True
    else:
        return False

def select_rand_point_on_map(startPose: Pose, costmap : OccupancyGrid) -> MoveBaseGoal:
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
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

def generate_random_target(distance_threshold=0):
    set_thresh_flag = False
    if (distance_threshold !=0):
        set_thresh_flag= True
    
    rospy.loginfo("Waiting for target_generator node to be available")
    rospy.wait_for_service('/target_generator/generate_nav_target')
    try:
        rand_target_service= rospy.ServiceProxy('/target_generator/generate_nav_target', GenerateTarget)
        response = rand_target_service(set_thresh_flag, distance_threshold)
        rospy.loginfo("Requested a random target from target_generator node")
        return response.goal
    except rospy.ServiceException as e:
        rospy.logerr("Service call for Generating Random Target failed: %s"%e)
    return None

    



def get_test_goal(robot_pose:PoseWithCovarianceStamped):
    # Creates a new goal with the MoveBaseGoal constructor
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    # Move ? meters forward along the x axis of the "map" coordinate frame 
    goal.target_pose.pose.position.x = robot_pose.position.x + 1
    # Move ? meters forward along the y axis of the "map" coordinate frame 
    goal.target_pose.pose.position.y = 0
    # No rotation of the mobile base frame w.r.t. map frame
    goal.target_pose.pose.orientation.w = robot_pose.orientation.w

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

def stop_in_place():
    print("Stopping in place")
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.linear.x=0
    vel_msg.linear.y=0
    vel_msg.linear.z=0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0 
    velocity_publisher.publish(vel_msg)

def angles_close(a, b, tolerance : float) -> bool:
    """
        a, b: angles in radians
        tolerance: tolerance of the check in degrees for simplicity
        returns True is a, b and within the tolerance range to eachother
    """
    tolerance_tf = np.deg2rad(tolerance)
    if abs(a - b) <= tolerance_tf:
        return True
    else:
        return False

def initiate_rotation():
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    speed = 30 #speed in deg/sec
    angular_speed = np.deg2rad(speed)

    vel_msg = Twist()
    #We wont use linear components
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = -abs(angular_speed)
    velocity_publisher.publish(vel_msg)


# call this on shutdown (ctrl-c)
def stop_motors():
    print('Stopping motors')
    #Forcing our robot to stop
    velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    velocity_publisher.publish(vel_msg)