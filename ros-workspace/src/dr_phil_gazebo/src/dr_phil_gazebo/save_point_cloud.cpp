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
#include <pcl/io/ply_io.h>


class SubscribeAndPublish
{

ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::Subscriber sub1_;
public:
SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<sensor_msgs::PointCloud2>("/filteredCloud", 1);

    //Topic you want to subscribe
    sub1_ = n_.subscribe("/pointCloud", 1, &SubscribeAndPublish::filteredOut, this);
    
    ros::Rate rate(30.0);
    while (n_.ok()){
        ros::spinOnce();
        rate.sleep();
    }
  }
void
filteredOut(sensor_msgs::PointCloud cloud_msg)
{

  sensor_msgs::PointCloud2 cloud_filtered;

  //convert to .ply to save and publish to topic
  pcl::PCLPointCloud2 pcl_pc2;
  sensor_msgs::convertPointCloudToPointCloud2(cloud_msg, cloud_filtered);
  pcl_conversions::toPCL(cloud_filtered, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
  pcl::io::savePLYFile("handle_coverage.ply",  *temp_cloud);
  pub_.publish(cloud_filtered);
}
};
int main(int argc, char **argv)
{


  //Initiate ROS
  ros::init(argc, argv, "filterCloud");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
