/*******************************************************************************
* Copyright 2020 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Ryan Shim */

#include "dr_phil_hardware_bringup/dr_phil_hardware_bringup.h"
#include <ros/console.h>
#include <std_msgs/MultiArrayDimension.h>

Turtlebot3ManipulationBringup::Turtlebot3ManipulationBringup()
: nh_(""),
  arm_action_server_(nh_, 
    "arm_controller/follow_joint_trajectory", 
    boost::bind(&Turtlebot3ManipulationBringup::armActionCallback, this, _1), 
    false)
{
  // Init Publisher
  joint_trajectory_point_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("joint_trajectory_point", 10);

  // Init subscriber
  gripper_pos_sub_ = nh_.subscribe("gripper_position",100,&Turtlebot3ManipulationBringup::gripperPosCallback,this);

  curr_state = std_msgs::Float64MultiArray();
  curr_state.data.resize(6);
  for(int i = 0; i < 6; i++){
    curr_state.data[i] = 0;
  }

  arm_action_server_.start();
}

Turtlebot3ManipulationBringup::~Turtlebot3ManipulationBringup() {}

void Turtlebot3ManipulationBringup::publishPoint(const std_msgs::Float64MultiArray msg){
  curr_state = msg;
  joint_trajectory_point_pub_.publish(msg);
}

void Turtlebot3ManipulationBringup::gripperPosCallback(const std_msgs::Float64 &msg){
  ROS_ERROR("%s",msg);
  gripper_state = msg;
  ROS_ERROR("%f",msg.data);
  curr_state.data[5] = msg.data;
  ROS_ERROR("%f",msg.data);
  publishPoint(curr_state);


}

void Turtlebot3ManipulationBringup::armActionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
  trajectory_msgs::JointTrajectory jnt_tra = goal->trajectory;

  bool success = true;
  uint32_t jnt_tra_pts_size = jnt_tra.points.size();
  const uint8_t POINTS_STEP_SIZE = 10;
  uint32_t steps = floor((double)jnt_tra_pts_size/(double)POINTS_STEP_SIZE);
  
  ros::Time start_time = jnt_tra.header.stamp;

  for (uint32_t i = 0; i < jnt_tra_pts_size; i++)// = i + steps)
  {
    if (arm_action_server_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", "arm_controller/follow_joint_trajectory");
      // set the action state to preempted
      arm_action_server_.setPreempted();
      success = false;
      break;
    }

    std_msgs::Float64MultiArray jnt_tra_pts;

    jnt_tra_pts.data.push_back(0); // imitate hardware joint_trajectory_point[0] is always zero
    for (std::vector<uint32_t>::size_type j = 0; j < jnt_tra.points[i].positions.size(); j++)
    {
      jnt_tra_pts.data.push_back(jnt_tra.points[i].positions[j]);
    }
    jnt_tra_pts.data.push_back(gripper_state.data); // gripper 
    publishPoint(jnt_tra_pts);

    // sleep if ahead of planned time
    ros::Time now = ros::Time::now();
    ros::Duration time_ahead = (start_time + jnt_tra.points[i].time_from_start) - now;
    if(time_ahead.toSec() > 0){
      time_ahead.sleep();
    }
    
  }

  if(success){
    ROS_INFO("%s: Succeeded", "arm_controller/follow_joint_trajectory");
    arm_action_server_.setSucceeded();
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "dr_phil_hardware_bringup");
  Turtlebot3ManipulationBringup turtlebot3_manipulation_bringup;
  
  ros::spin();
  return 0;
}
