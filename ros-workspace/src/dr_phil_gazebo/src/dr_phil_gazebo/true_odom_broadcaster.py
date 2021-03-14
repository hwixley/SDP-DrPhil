#!/usr/bin/env python3


from rospy.impl.tcpros_service import ServiceProxy
import rospy
import tf_conversions
import tf2_ros
import geometry_msgs.msg 

from gazebo_msgs.srv import GetModelState,GetModelStateResponse




def true_pose(pose_service : ServiceProxy):

    true_pose : GetModelStateResponse = pose_service.call("dr-phil","")
    
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_footprint"
    t.transform.translation.x = true_pose.pose.position.x 
    t.transform.translation.y = true_pose.pose.position.y
    t.transform.translation.z = true_pose.pose.position.z 
    t.transform.rotation = true_pose.pose.orientation

    br.sendTransform(t)

if __name__ =="__main__":
    rospy.init_node("true_pose_broadcaster")

    r = rospy.Rate(100)
    s = rospy.ServiceProxy("/gazebo/get_model_state",GetModelState,persistent=True)

    while not rospy.is_shutdown():
        true_pose(s)
        r.sleep()


        

