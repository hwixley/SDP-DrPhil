#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sstream>
#include <std_msgs/Float32.h>
#include <mutex>         
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "geometry_msgs/TransformStamped.h"

#define FIXED_FRAME "map"

class SubscribeAndPublish
{

sensor_msgs::PointCloud MergedCloud;
int n=0;
tf2_ros::Buffer *tfBuffer;
tf2_ros::TransformListener *tfListener;
std::mutex mutex_;
float history_period = 10;

public:


  void publish_merged(){
    std::unique_lock<std::mutex> lck(mutex_);
    if (MergedCloud.points.size() > 0){
      MergedCloud.header.frame_id = FIXED_FRAME;
      pub_.publish(MergedCloud);
    } else {
      ROS_INFO_THROTTLE(1,"empty merged cloud");
    }
    lck.unlock();
  }

  SubscribeAndPublish()
  {
    // keep history of transforms up to 10 seconds
    tfBuffer = new tf2_ros::Buffer(ros::Duration(history_period));
    tfListener = new tf2_ros::TransformListener(*tfBuffer);

    pub_ = n_.advertise<sensor_msgs::PointCloud>("/pointCloud", 10);
    sub1_ = n_.subscribe("/spray_controller/command", 100, &SubscribeAndPublish::callback1, this);
    

    // publish merged occasionaly
    ros::Rate r(20);

    while(ros::ok()){
      publish_merged();
      ros::spinOnce();
      r.sleep();
    }

  }

  void callback1(const std_msgs::Float32 &msg_range)
  {
    if (msg_range.data ==1){
      ROS_INFO("Enabling nozzle subscribers");
      sub_ = n_.subscribe("/depth_camera/depth/points", 3, &SubscribeAndPublish::merge_callback, this);
      subL_ = n_.subscribe("/depth_left/depth/points", 3, &SubscribeAndPublish::merge_callback, this);
      subR_ = n_.subscribe("/depth_right/depth/points", 3, &SubscribeAndPublish::merge_callback, this);
    }
    
    if (msg_range.data ==0){
      ROS_INFO("Disabling nozzle subscribers");
      sub_.shutdown();
      subL_.shutdown();
      subR_.shutdown();

    }
  }
  
  



  void merge_callback(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input){

    // if cloud is older than we have transforms for
    if (input->header.stamp <= (ros::Time::now() - ros::Duration(history_period))){
        ROS_INFO("Ignoring point cloud older than 10 seconds");
        return;
    }

    // tf::StampedTransform transform;
    geometry_msgs::TransformStamped transformStamped;

    ROS_INFO("cloud arrived from timestamp: %f",input->header.stamp.toSec());
    try{
        // tfBuffer->waitForTransform(FIXED_FRAME, input->header.frame_id, input->header.stamp, ros::Duration(5));
        transformStamped = tfBuffer->lookupTransform(FIXED_FRAME,input->header.frame_id , input->header.stamp);

    }
    catch (tf2::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
    sensor_msgs::PointCloud2 cloud_publish;
    tf2::doTransform((sensor_msgs::PointCloud2)*input, cloud_publish,transformStamped);

    
    //convert to old pointcloud type to merge latest pointcloud with 
    //larger cloud being built up
    sensor_msgs::PointCloud inputCloud; 
    sensor_msgs::convertPointCloud2ToPointCloud(cloud_publish, inputCloud);

    inputCloud.header.frame_id = input->header.frame_id;
    inputCloud.header.stamp = input->header.stamp;

    for (int p=0; p<inputCloud.points.size(); ++p)
    {
      if (isnan(inputCloud.points[p].x))
      {
        inputCloud.points[p] = inputCloud.points[inputCloud.points.size()-1];
        inputCloud.points.resize(inputCloud.points.size()-1);
        --p;
      }
    }

    ROS_INFO_THROTTLE(0.5,"new cloud points: %lu, frame: %s",inputCloud.points.size(),input->header.frame_id.c_str());
    // prevent other threads modifying merged cloud 
    std::unique_lock<std::mutex> lck(mutex_);

    if(n==0){
      MergedCloud = inputCloud;
    }
 
    //publish to topic
    sensor_msgs::PointCloud tempCloud;  
    merge_point_cloud(inputCloud, MergedCloud,tempCloud);
    MergedCloud = tempCloud;

    // now fair game to modify
    lck.unlock();

    n++;

  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Subscriber sub1_;
  ros::Subscriber subL_;
  ros::Subscriber subR_;



  void merge_point32_vector(const std::vector<geometry_msgs::Point32>& vin, std::vector<geometry_msgs::Point32>& vout)
{
    vout.reserve(vout.size() + vin.size());
    vout.insert(vout.end(), vin.begin(), vin.end());
    std::sort(vout.begin(), vout.end(), [](geometry_msgs::Point32 p1, geometry_msgs::Point32 p2){
if (p1.x != p2.x)
    return p1.x > p2.x;
else if (p1.y != p2.y)
    return  p1.y > p2.y;
else
    return p1.z > p2.z;
});
    auto unique_end = std::unique(vout.begin(), vout.end(), [](geometry_msgs::Point32 p1, geometry_msgs::Point32 p2){
    if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z)
        return true;
    return false;
});
    vout.erase(unique_end, vout.end());
}

/**
 * \brief Function to merge two PointCloud data after checking if they are in they have the same `frame_id`.
 * \param cloud_in1, cloud_in2 : Two input PointClouds
 * \param cloud_out : Output PointCloud
 * \return true if succesful and false otherwise
 */
bool merge_point_cloud(const sensor_msgs::PointCloud& cloud_in1, const sensor_msgs::PointCloud& cloud_in2, sensor_msgs::PointCloud& cloud_out)
{
    // check if both have the same frame_id
    // if(cloud_in1.header.frame_id != cloud_in2.header.frame_id)
    //     return false;
    // set cloud_out frame_id
    cloud_out.header.frame_id = FIXED_FRAME;
    // merge the clouds one by one
    merge_point32_vector(cloud_in1.points, cloud_out.points);
    merge_point32_vector(cloud_in2.points, cloud_out.points);
    return true;
}

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  return 0;
}
