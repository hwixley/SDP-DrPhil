#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
import dr_phil_hardware.vision.explore as explore

class MapExplorer:
    def __init__(self):
        self.global_costmap = None
        self.global_costmap_sub = rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, callback=self.global_costmap_callback)

    def global_costmap_callback(self, global_costmap : OccupancyGrid):
        self.global_costmap = global_costmap

    def spin(self):
        pass

if __name__ == "__main__":
    rospy.init_node("map_explorer", anonymous=False)
    #mapExplorer = MapExplorer()
    goal = explore.get_test_goal()
    result = explore.move_to_goal(goal)

    if result:
        rospy.loginfo("Goal execution done!")

    """
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        mapExplorer.spin()
        rate.sleep()
    """