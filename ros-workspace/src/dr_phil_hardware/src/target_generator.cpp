//taken from: https://github.com/surfertas/turtlepi/blob/master/turtlepi_navigate/src/target_generator.cpp

#include <target_generator/target_generator.h>


using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::FREE_SPACE;

TargetGenerator::TargetGenerator(ros::NodeHandle& nh)
: nh_(nh)
{

  registerService();

  registerSubscriber();
  // if (map_size_x_) {
  //   std::cout << "Cost map properly initialized." << std::endl;
  //   init_ = true;
  // } else {
  //   std::cout << "Issues initializing cost map." << std::endl;
  // }
  setParams();
  
  registerPublisher();

}

TargetGenerator::~TargetGenerator()
{
  srv_generate_target_.shutdown();
}

void
TargetGenerator::registerSubscriber()
{
  sub_turtlepi_location_ = nh_.subscribe("/amcl_pose", 10, &TargetGenerator::currentPositionCB, this);
  sub_costmap = nh_.subscribe("/move_base/global_costmap/costmap", 10, &TargetGenerator::costMapInit, this);
  // sub_local_costmap = nh_.subscribe("/move_base/local_costmap/costmap", 10, &TargetGenerator::localCostMapInit, this);
  init_ = true;

  std::cout << "Registered subscriber." << std::endl;
}

void
TargetGenerator::registerPublisher()
{
  pub_visualization_marker_ = nh_.advertise<visualization_msgs::Marker>("/turtlepi_navigate/visualization_marker", 0);
}

void
TargetGenerator::registerService()
{
  srv_generate_target_ =
      nh_.advertiseService("/target_generator/generate_nav_target", &TargetGenerator::generateTargetService, this);
  std::cout << "Registered generate target server." << std::endl;
}

void
TargetGenerator::currentPositionCB(
  const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& location)
{
  current_position_ = *location;

}


void
TargetGenerator::costMapInit(const nav_msgs::OccupancyGrid costmap)
{
  // while (!ros::service::waitForService("static_map", ros::Duration(-1))) {
  //   std::cout << "Waiting for costmap" << std::endl;
  //   ros::Duration(2).sleep();
  // }

  // ros::ServiceClient map_service_client = nh_.serviceClient<nav_msgs::GetMap>("static_map");
  // nav_msgs::GetMap srv_map;


  // if (map_service_client.call(srv_map)) {
    map_origin_x_ = costmap.info.origin.position.x;
    map_origin_y_ = costmap.info.origin.position.y;
    map_resolution_ = costmap.info.resolution;
    map_size_x_ = costmap.info.width;
    map_size_y_ = costmap.info.height;
    map_data_ = costmap.data;

}






void
TargetGenerator::localCostMapInit(const nav_msgs::OccupancyGrid local_costmap)
{
    local_map_origin_x_ = local_costmap.info.origin.position.x;
    local_map_origin_y_ = local_costmap.info.origin.position.y;
    local_map_resolution_ = local_costmap.info.resolution;
    local_map_size_x_ = local_costmap.info.width;
    local_map_size_y_ = local_costmap.info.height;
    local_map_data_ = local_costmap.data;

}

//If possible, checks in 3x3 square the pixels surrounding (x,y) coordinates, starting from "top-left"
bool 
TargetGenerator::check_local_surrounding(uint32_t localmap_x, uint32_t localmap_y){
    if ( (localmap_x - 1 > 0 && localmap_x + 1 < local_map_size_x_) && (localmap_y-1 > 0 && localmap_y +1 < local_map_size_y_)){
        for (int i=-1; i<=1; i++){
          for (int j=-1; j<=1; j++){
            uint32_t idx = (localmap_x + i) + (localmap_y + j) * local_map_size_x_;
            //If one of the points is not free (hence, any number but 0), return false

            if (local_map_data_[idx] != 0) {
              return false;
            }
          }
        }
        //Since all the surrounding space is free, return true 
        return true;
    }
    //We assume the surrounding is free if we cant read the data. In other words, if the target point generated is outside the range of the data we have on local map.
    return true;
}

// //If possible, checks in 9x9 square the pixels surrounding (x,y) coordinates, starting from "top-left"
bool 
TargetGenerator::check_global_surrounding(uint32_t map_x, uint32_t map_y){
    // uint32_t map_x;
    // uint32_t map_y;
    if ( (map_x - 4 > 0 && map_x + 4 < map_size_x_) && (map_y- 4 > 0 && map_y +4 < map_size_y_)){
        for (int i=-4; i<=4; i++){
          for (int j=-4; j<=4; j++){
            //worldToMap(map_x,)
            uint32_t idx = (map_x + i) + (map_y + j) * map_size_x_;
            //If one of the points is not free (hence, any number but 0), return false
            if (map_data_[idx] != 0) {
              return false;
            }
          }
        }
        //Since all the surrounding space is free, return true 
        return true;
    }
    //we shouldnt take the edge of the map, return false
    return false;
}






bool
TargetGenerator::generateTargetService(
  dr_phil_hardware::GenerateTarget::Request& req,
  dr_phil_hardware::GenerateTarget::Response& res)

{ 
  //If the parameter is set of custom distance threshold is true, change the value to the one passed in the call
  if (req.set_custom_distance_threshold){
    DISTANCE_THRESHOLD_ = req.distance_thresh;
  }


  std::random_device rd;
  std::mt19937 gen(rd());

  // Use 5 as buffer for map boundaries, as want to avoid edge cases.
  std::uniform_int_distribution<> grid_x(5, map_size_x_ - 5);
  std::uniform_int_distribution<> grid_y(5, map_size_y_ - 5);


  double world_x, world_y;
  uint32_t idx;
  uint32_t local_idx;

  bool thresh;
  bool is_global_free;
  bool is_free;

  auto checkThresh = [&](double x, double y, double wx, double wy) {
    return sqrt(pow(wx - x, 2) + pow(wy - y, 2)) >= DISTANCE_THRESHOLD_ && sqrt(pow(wx - x, 2) + pow(wy - y, 2)) <= DISTANCE_THRESHOLD_ + 1; //Must be lower than the threshold
  };

  auto printTarget = [](double wx, double wy) {
    std::cout << "Target: (" << wx << " ," << wy << ")" << std::endl;
  };


  
  // int count_iterations = 0;
  // auto original_threshold = DISTANCE_THRESHOLD;
  do {
    uint32_t localmap_x;
    uint32_t localmap_y;

    uint32_t map_x = grid_x(gen);
    uint32_t map_y = grid_y(gen);

    

    mapToWorld(map_x, map_y, world_x, world_y);
    // worldToLocalMap(localmap_x,localmap_y,world_x,world_y);



    idx = map_x + map_y * map_size_x_;
    // local_idx = localmap_x + localmap_y * local_map_size_x_;

    thresh = checkThresh(current_position_.pose.pose.position.x,
                    current_position_.pose.pose.position.y,
                    world_x,
                    world_y);


    //If we cannot determine the local costmap (point generated is further than local costmap size), assume it is free
    // if (local_idx > local_map_data_.size() && map_data_[idx] == 0){
    //   is_free = 1;
    // }
    // //check if it is free when target is within range
    // else if(local_map_data_[local_idx] == 0 && map_data_[idx] == 0){
    //   // is_free = check_local_surrounding(world_x,world_y);
    // }
    // else{
    //   is_free = 0;
    // }

    is_global_free = check_global_surrounding(map_x,map_y);


  } while (! ((is_global_free==1 && (thresh == 1))));

  double radians = theta_ * (PI_ / 180.0);
  tf::Quaternion quaternion;
  quaternion = tf::createQuaternionFromYaw(radians);
  geometry_msgs::Quaternion q_msg;
  tf::quaternionTFToMsg(quaternion, q_msg);

  res.goal.target_pose.header.frame_id = "map";
  res.goal.target_pose.header.stamp = ros::Time::now();
  res.goal.target_pose.pose.position.x = world_x;
  res.goal.target_pose.pose.position.y = world_y;
  res.goal.target_pose.pose.orientation = q_msg;
  res.success = true;
  printTarget(world_x, world_y);
  targetMarker(world_x, world_y);




  return res.success;
}

void
TargetGenerator::mapToWorld(
  const uint32_t map_x,
  const uint32_t map_y,
  double& world_x,
  double& world_y) const
{
  world_x = map_origin_x_ + (map_x +0.5) * map_resolution_;
  world_y = map_origin_y_ + (map_y +0.5) * map_resolution_;
}


void
TargetGenerator::worldToMap(
  uint32_t& map_x,
  uint32_t& map_y,
  const double world_x,
  const double world_y) const
{
  map_x = (uint32_t)((world_x - map_origin_x_) / map_resolution_);
  map_y = (uint32_t)((world_y - map_origin_y_) / map_resolution_);
}

void
TargetGenerator::worldToLocalMap(
  uint32_t& map_x,
  uint32_t& map_y,
  const double world_x,
  const double world_y) const
{
  map_x = (uint32_t)((world_x - local_map_origin_x_) / local_map_resolution_);
  map_y = (uint32_t)((world_y - local_map_origin_y_) / local_map_resolution_);
}

void
TargetGenerator::targetMarker(const double x, const double y) const
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "turtlepi_navigate";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.lifetime = ros::Duration(100);
  pub_visualization_marker_.publish(marker);
}


void
TargetGenerator::setParams()
{
  // set target orientation of robot.
  theta_ = 90.0;
  PI_ = 3.14159265358;
  DISTANCE_THRESHOLD_ = 1.0; //Setting the threshold low 
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_generator");
  ros::NodeHandle nh;

  TargetGenerator generator(nh);
  
  ros::spin();
  return 0;
}