#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"


#include <gazebo/gazebo_client.hh>


void sprayCallback(const std_msgs::Float32 &msg_range)
{
  // Load gazebo as a client
  gazebo::client::setup();

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  
  // Publish to the  light/modify gazebo topic
  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::Light>("~/light/modify");
    
    // Wait for a subscriber to connect to this publisher
  pub->WaitForConnection();
  
  // Create a lightmessage
  gazebo::msgs::Light msg;
  msg.set_name("dr-phil::px100/gripper_link::led");
  msg.set_range(float(msg_range.data));

  // Send the message
  pub->Publish(msg);
  
  gazebo::client::shutdown();
 // printf("I heard: [%s]", msg_range->data.c_str());
}
/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  
  // Load gazebo as a client
  /*gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  
  // Publish to the  light/modify gazebo topic
  gazebo::transport::PublisherPtr pub =
    node->Advertise<gazebo::msgs::Light>("~/light/modify");
    
    // Wait for a subscriber to connect to this publisher
  pub->WaitForConnection();
  
  // Create a lightmessage
  gazebo::msgs::Light msg;
  msg.set_name("dr-phil::px100/gripper_link::led");
  msg.set_range(1);

  // Send the message
  pub->Publish(msg);
  
  gazebo::client::shutdown();*/
  ros::init(_argc, _argv, "spray_listener");
  
  ros::NodeHandle nh;
  
  ros::Subscriber sub = nh.subscribe("/spray_controller/command", 1000, sprayCallback);
  
  ros::spin();
  
  
  return 0;




}
