// Copyright 2024 Andrzej_Norbert_Jeremiasz
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "obstacle_dstar/obstacle_dstar_node.hpp"


namespace obstacle_dstar
{

PoseArray trajectory2PoseArray(const Trajectory & trajectory)
{
  PoseArray pose_array;
  pose_array.header = trajectory.header;

  for (const auto & point : trajectory.points) {
    pose_array.poses.push_back(point.pose);
  }

  return pose_array;
}

Trajectory getPartialTrajectory(
  const Trajectory & trajectory, const size_t start_index, const size_t length)
{

  Trajectory partial_trajectory;
  partial_trajectory.header = trajectory.header;
  partial_trajectory.header.stamp = rclcpp::Clock().now();

  partial_trajectory.points.reserve(length);
  for (size_t i = 0; i < length; ++i) {
    const size_t index = (start_index + i) % trajectory.points.size();
    partial_trajectory.points.push_back(trajectory.points.at(index));
  }

  return partial_trajectory;
}

Trajectory createTrajectory(
  const PoseStamped & current_pose, const PlannerWaypoints & planner_waypoints,
  const double & velocity)
{
  Trajectory trajectory;
  trajectory.header = planner_waypoints.header;

  for (const auto & awp : planner_waypoints.waypoints) {
    TrajectoryPoint point;

    point.pose = awp.pose.pose;

    point.pose.position.z = current_pose.pose.position.z;  // height = const
    point.longitudinal_velocity_mps = velocity / 3.6;      // velocity = const

    trajectory.points.push_back(point);
  }

  return trajectory;
}

Trajectory replaceTrajectorySegment(const Trajectory & original_trajectory, const Trajectory & new_trajectory, const size_t start_index)
{
  Trajectory modified_trajectory = original_trajectory;

  if (start_index + 25 <= modified_trajectory.points.size()) {
    modified_trajectory.points.erase(modified_trajectory.points.begin() + start_index, modified_trajectory.points.begin() + start_index + 25);
  } else {
    size_t points_to_remove = 25 - (modified_trajectory.points.size() - start_index);
    modified_trajectory.points.erase(modified_trajectory.points.begin() + start_index, modified_trajectory.points.end());
    if (points_to_remove > 0 ) {
      modified_trajectory.points.erase(modified_trajectory.points.begin(), modified_trajectory.points.begin() + points_to_remove);
    } 
  }
  modified_trajectory.points.insert(modified_trajectory.points.begin() + start_index, new_trajectory.points.begin(), new_trajectory.points.end());
  
  return modified_trajectory;
}


ObstacleDstarNode::ObstacleDstarNode(const rclcpp::NodeOptions & node_options)
: Node("obstacle_dstar", node_options)
{
  using std::placeholders::_1;

  // NodeParam
  {
    auto & p = node_param_;
    p.planning_algorithm = declare_parameter<std::string>("planning_algorithm");
    p.waypoints_velocity = declare_parameter<double>("waypoints_velocity");
    p.update_rate = declare_parameter<double>("update_rate");
    p.th_arrived_distance_m = declare_parameter<double>("th_arrived_distance_m");
    p.th_stopped_time_sec = declare_parameter<double>("th_stopped_time_sec");
    p.th_stopped_velocity_mps = declare_parameter<double>("th_stopped_velocity_mps");
    p.th_course_out_distance_m = declare_parameter<double>("th_course_out_distance_m");
    p.vehicle_shape_margin_m = declare_parameter<double>("vehicle_shape_margin_m");

    

  }

  // set vehicle_info
  {
    const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
    vehicle_shape_.length = vehicle_info.vehicle_length_m;
    vehicle_shape_.width = vehicle_info.vehicle_width_m;
    vehicle_shape_.base2back = vehicle_info.rear_overhang_m;
  }

  // Planning
  initializePlanningAlgorithm();

  // Subscribers
  {
      trajectory_sub_ = create_subscription<Trajectory>(
      "~/input/trajectory", rclcpp::QoS{1},
      std::bind(&ObstacleDstarNode::onTrajectory, this, _1));
    occupancy_grid_sub_ = create_subscription<OccupancyGrid>(
      "~/input/occupancy_grid", 1, std::bind(&ObstacleDstarNode::onOccupancyGrid, this, _1));
    odom_sub_ = create_subscription<Odometry>(
      "~/input/odometry", 100, std::bind(&ObstacleDstarNode::onOdometry, this, _1));
  }

  // Publishers
  {
    rclcpp::QoS qos{1};
    qos.transient_local();  // latch
    trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", qos);
    occupancy_grid_pub_ = create_publisher<OccupancyGrid>("~/output/partial_grid", qos);

    markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("~/output/markers", rclcpp::QoS(1000));

  }

  // Timer
  {
    const auto period_ns = rclcpp::Rate(node_param_.update_rate).period();
    timer_ = rclcpp::create_timer(
      this, get_clock(), period_ns, std::bind(&ObstacleDstarNode::onTimer, this));
  }

  logger_configure_ = std::make_unique<tier4_autoware_utils::LoggerLevelConfigure>(this);
}

PlannerCommonParam ObstacleDstarNode::getPlannerCommonParam()
{
  PlannerCommonParam p;

  // search configs
  p.time_limit = declare_parameter<double>("time_limit");
  p.minimum_turning_radius = declare_parameter<double>("minimum_turning_radius");
  p.maximum_turning_radius = declare_parameter<double>("maximum_turning_radius");
  p.turning_radius_size = declare_parameter<int>("turning_radius_size");
  p.maximum_turning_radius = std::max(p.maximum_turning_radius, p.minimum_turning_radius);
  p.turning_radius_size = std::max(p.turning_radius_size, 1);

  p.theta_size = declare_parameter<int>("theta_size");
  p.angle_goal_range = declare_parameter<double>("angle_goal_range");
  p.curve_weight = declare_parameter<double>("curve_weight");
  p.reverse_weight = declare_parameter<double>("reverse_weight");
  p.lateral_goal_range = declare_parameter<double>("lateral_goal_range");
  p.longitudinal_goal_range = declare_parameter<double>("longitudinal_goal_range");

  // costmap configs
  p.obstacle_threshold = declare_parameter<int>("obstacle_threshold");

  return p;
}

void ObstacleDstarNode::onTrajectory(const Trajectory::ConstSharedPtr msg)
{
  input_trajectory_ = msg;
  goal_pose_.header = msg->header;
  
}

void ObstacleDstarNode::onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg)
{
  occupancy_grid_ = msg;
}

void ObstacleDstarNode::onOdometry(const Odometry::ConstSharedPtr msg)
{
  odom_ = msg;
  odom_buffer_.push_back(msg);

  // Delete old data in buffer
  while (true) {
    const auto time_diff =
      rclcpp::Time(msg->header.stamp) - rclcpp::Time(odom_buffer_.front()->header.stamp);

    if (time_diff.seconds() < node_param_.th_stopped_time_sec) {
      break;
    }

    odom_buffer_.pop_front();
  }
}

bool ObstacleDstarNode::detectCollision()
  {
    extractPartialGridMap();
    if (modified_trajectory_.points.size() <= 1) {
      return true;
    }


    for (const auto &point : modified_trajectory_.points)
    {
      // Check the surrounding area 5x5 on grid map for collision
      const auto pose = point.pose;
      int nearest_x = (pose.position.x - occupancy_grid_->info.origin.position.x )/occupancy_grid_->info.resolution;
      int nearest_y = (pose.position.y - occupancy_grid_->info.origin.position.y )/occupancy_grid_->info.resolution;
      std::vector<int> surrounding_indices;
      const int row_num = nearest_y;
      const int col_num = nearest_x;

      for (int i = -2; i <= 2; i++)
      { for (int j = -2; j <= 2; j++)
        {
          const int index = (row_num + i) * occupancy_grid_->info.width + col_num + j;
          surrounding_indices.push_back(index);
        }
      }
      
      for (const auto &index : surrounding_indices)
      {
        if (occupancy_grid_->data.at(index) > node_param_.obstacle_threshold)
        {
          return true;
        }
      }
    }
    return false;
}

void ObstacleDstarNode::onTimer()
{
  // Check all inputs are ready
  if (!occupancy_grid_ || !input_trajectory_ || !odom_) {
    return;
  }
  // Get current pose
  current_pose_.pose = odom_->pose.pose;
  current_pose_.header = odom_->header;

  if (current_pose_.header.frame_id == "") {
    return;
  }

  if(isPlanRequired()) // detectCollision()
  {
    if (is_detected){

      planTrajectory();
    }
    else {
      is_detected = true;
      algo_->setMap(*partial_grid_);
      if (auto dstar_algo = dynamic_cast<DstarSearch*>(algo_.get())) {
        dstar_algo->clearNodesDstar();
      } else {
        throw std::runtime_error("Algorithm is not DstarSearch");
      }      
      }
  } else {
    is_detected = false;
  }
  

  trajectory_pub_->publish(modified_trajectory_);

}

void ObstacleDstarNode::extractPartialGridMap()
{
  PoseStamped internal_goal = PoseStamped();
  if (!modified_trajectory_.points.empty()){
  const size_t start_index_forward = motion_utils::findNearestIndex(modified_trajectory_.points, current_pose_.pose.position);
  const size_t goal_index = (start_index_forward + 30) % modified_trajectory_.points.size();
  internal_goal.pose = modified_trajectory_.points[goal_index].pose;
      
  // Rectangle dimensions
  double width = 220;
  double height = 130;
  // Calculate the center of the rectangle
  double centerX = (current_pose_.pose.position.x + internal_goal.pose.position.x) / 2;
  double centerY = (current_pose_.pose.position.y + internal_goal.pose.position.y) / 2;

  // Occupancy grid parameters
  double resolution = occupancy_grid_->info.resolution;
  double originX = occupancy_grid_->info.origin.position.x;
  double originY = occupancy_grid_->info.origin.position.y;
  int gridWidth = occupancy_grid_->info.width;
  int gridHeight = occupancy_grid_->info.height;

  // Create a new occupancy grid for the partial map
  nav_msgs::msg::OccupancyGrid partial_grid_map;
  partial_grid_map.header = occupancy_grid_->header;
  partial_grid_map.info.resolution = resolution;
  partial_grid_map.info.width = width;
  partial_grid_map.info.height = height;
  partial_grid_map.info.origin.position.x = centerX - (width * resolution) / 2;
  partial_grid_map.info.origin.position.y = centerY - (height * resolution) / 2;
  partial_grid_map.data.resize(width * height, -1);

  // Extract the partial grid map from the original occupancy grid
  for (int i = 0; i < width; ++i) {
    for (int j = 0; j < height; ++j) {
      // Partial grid point in global coordinates
      double globalX = partial_grid_map.info.origin.position.x + i * resolution;
      double globalY = partial_grid_map.info.origin.position.y + j * resolution;

      // Convert to original grid indices
      int gridX = (globalX - originX) / resolution;
      int gridY = (globalY - originY) / resolution;

      // Check if indices are within bounds
      if (gridX >= 0 && gridX < gridWidth && gridY >= 0 && gridY < gridHeight) {
        int original_index = gridY * gridWidth + gridX;
        int partial_index = j * width + i;
        partial_grid_map.data[partial_index] = occupancy_grid_->data[original_index];
      }
    }
  }

  // Publish the partial grid map
  partial_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(partial_grid_map);
      } else {
        partial_grid_ = occupancy_grid_;
      }
  occupancy_grid_pub_->publish(*partial_grid_);
}

bool ObstacleDstarNode::isPlanRequired()
{
  if (modified_trajectory_.points.empty()){
    modified_trajectory_ = *input_trajectory_;
  }
  extractPartialGridMap();
  algo_->setMap(*partial_grid_);
  const size_t length_forward = 20; 
  const size_t start_index_forward =
    motion_utils::findNearestIndex(modified_trajectory_.points, current_pose_.pose.position);
  const auto forward_trajectory = getPartialTrajectory(modified_trajectory_, start_index_forward, length_forward);

  const bool is_obstacle_found = algo_->hasObstacleOnTrajectory(trajectory2PoseArray(forward_trajectory));
  if (is_obstacle_found) {
    return true;
  }
  return false;
}

void ObstacleDstarNode::planTrajectory()
{
  if (partial_grid_ == nullptr) {
    return;
  }
  // Provide robot shape and map for the planner
  algo_->setMap(*partial_grid_);  

  const rclcpp::Time start = get_clock()->now();
  const bool result = algo_->makePlan(current_pose_.pose, goal_pose_.pose);
  const rclcpp::Time end = get_clock()->now();

  visualization_msgs::msg::MarkerArray marker_array;
  marker_array = algo_->getMarkerArray();
  markers_pub_->publish(marker_array);

  if (result) {
    const size_t start_index_forward = motion_utils::findNearestIndex(input_trajectory_->points, current_pose_.pose.position);
    Trajectory new_trajectory =
        createTrajectory(current_pose_, algo_->getWaypoints(), node_param_.waypoints_velocity);

    modified_trajectory_ = replaceTrajectorySegment(modified_trajectory_, new_trajectory, start_index_forward);

  } 
}


void ObstacleDstarNode::initializePlanningAlgorithm()
{
  // Extend robot shape
  freespace_planning_algorithms::VehicleShape extended_vehicle_shape = vehicle_shape_;
  const double margin = node_param_.vehicle_shape_margin_m;
  extended_vehicle_shape.length += margin;
  extended_vehicle_shape.width += margin;
  extended_vehicle_shape.base2back += margin / 2;

  const auto planner_common_param = getPlannerCommonParam();

  const auto algo_name = node_param_.planning_algorithm;

  // initialize specified algorithm
  if (algo_name == "dstar") {
    algo_ = std::make_unique<DstarSearch>(planner_common_param, extended_vehicle_shape, *this);
  } else {
    throw std::runtime_error("No such algorithm named " + algo_name + " exists.");
  }
  RCLCPP_INFO_STREAM(get_logger(), "initialize planning algorithm: " << algo_name);
}
}  // namespace obstacle_dstar

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_dstar::ObstacleDstarNode)

