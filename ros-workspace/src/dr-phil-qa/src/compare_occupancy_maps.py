import rospy
from nav_msgs.msg import OccupancyGrid


class MapEvaluator():
    """ node which compares a map message against another taken as the ground truth,
    and performs quantative measurements of mapping quality """
    
    def __init__(self,measured_topic,truth_topic):
        self.measured_topic = measured_topic
        self.truth_topic = truth_topic
        

if __name__ == "__main__":
    rospy.init_node("map_evaluator")

    measured_topic = rospy.get_param("~map_evaluator/measured","/map")
    truth_topic = rospy.get_param("~map_evaluator/truth","/map2d")
    file_path = rospy.get_param("~map_evaluator/file_path","/data.csv")

    node = MapEvaluator(measured_topic,
                    truth_topic)

    pass