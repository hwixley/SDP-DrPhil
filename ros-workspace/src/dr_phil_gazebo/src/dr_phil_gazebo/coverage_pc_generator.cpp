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

class SubscribeAndPublish
{

sensor_msgs::PointCloud MergedCloud;
int n=0;
tf::StampedTransform transform;


public:
  SubscribeAndPublish()
  {
    
    pub_ = n_.advertise<sensor_msgs::PointCloud>("/pointCloud", 10);
    sub1_ = n_.subscribe("/spray_controller/command", 1, &SubscribeAndPublish::callback1, this);
    
    
  
  }

  void callback1(const std_msgs::Float32 &msg_range)
  {
    if (msg_range.data ==1){
  
      sub_ = n_.subscribe("/depth_camera/depth/points", 10, &SubscribeAndPublish::callback, this);
    }
    
    if (msg_range.data ==0){
      sub_.shutdown();
    }
  }
  
  
  void callback(const boost::shared_ptr<const sensor_msgs::PointCloud2> &input)
  {

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud inputCloud;
    sensor_msgs::PointCloud tempCloud;   
    sensor_msgs::PointCloud2 test;
    
    //convert to PCL pointcloud then transform to global frame
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

    //find transform from camera frame to global
    tf::TransformListener listener;

    try{
        listener.waitForTransform("odom", "px100/led_link1", ros::Time::now(), ros::Duration(5));
        listener.lookupTransform("odom", "px100/led_link1", ros::Time(), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }


    pcl_ros::transformPointCloud(*temp_cloud, *cloud_transformed, transform);
    sensor_msgs::PointCloud2 cloud_publish;
    pcl::toROSMsg(*cloud_transformed,cloud_publish);

    
    //convert to old pointcloud type to merge latest pointcloud with 
    //larger cloud being built up
    sensor_msgs::convertPointCloud2ToPointCloud(cloud_publish, inputCloud);
    

    for (int p=0; p<inputCloud.points.size(); ++p)
    {
      if (isnan(inputCloud.points[p].x))
      {
        inputCloud.points[p] = inputCloud.points[inputCloud.points.size()-1];
        inputCloud.points.resize(inputCloud.points.size()-1);
        --p;
      }
    }
    
    if(n==0){
      MergedCloud = inputCloud;
    }

    //publish to topic
    merge_point_cloud(inputCloud, MergedCloud,tempCloud);
    MergedCloud = tempCloud;
    pub_.publish(MergedCloud);
    n++;
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Subscriber sub1_;



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
bool comparePoint(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2){
if (p1.x != p2.x)
    return p1.x > p2.x;
else if (p1.y != p2.y)
    return  p1.y > p2.y;
else
    return p1.z > p2.z;
}

bool equalPoint(geometry_msgs::Point32 p1, geometry_msgs::Point32 p2){
    if (p1.x == p2.x && p1.y == p2.y && p1.z == p2.z)
        return true;
    return false;
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
    if(cloud_in1.header.frame_id != cloud_in2.header.frame_id)
        return false;
    // set cloud_out frame_id
    cloud_out.header.frame_id = cloud_in1.header.frame_id;
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

  ros::spin();

  return 0;
}
