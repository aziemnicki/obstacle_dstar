// // Copyright 2024 Andrzej_Norbert_Jeremiasz
// //
// // Licensed under the Apache License, Version 2.0 (the "License");
// // you may not use this file except in compliance with the License.
// // You may obtain a copy of the License at
// //
// //     http://www.apache.org/licenses/LICENSE-2.0
// //
// // Unless required by applicable law or agreed to in writing, software
// // distributed under the License is distributed on an "AS IS" BASIS,
// // WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// // See the License for the specific language governing permissions and
// // limitations under the License.

// #include "obstacle_dstar/obstacle_dstar_node.hpp"
// #include "freespace_planning_algorithms/abstract_algorithm.hpp"

// #include <motion_utils/trajectory/trajectory.hpp>
// #include <tier4_autoware_utils/geometry/geometry.hpp>

// #include <algorithm>
// #include <deque>
// #include <memory>
// #include <string>
// #include <utility>
// #include <vector>
// #include <typeinfo>

// namespace
// {
// using autoware_auto_planning_msgs::msg::Trajectory;
// using autoware_auto_planning_msgs::msg::TrajectoryPoint;
// using TrajectoryPoints = std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint>;
// using freespace_planning_algorithms::AstarSearch;
// using freespace_planning_algorithms::Dstar;
// using freespace_planning_algorithms::DstarParam;
// using freespace_planning_algorithms::PlannerWaypoint;
// using freespace_planning_algorithms::PlannerWaypoints;
// using freespace_planning_algorithms::RRTStar;
// using geometry_msgs::msg::Pose;
// using geometry_msgs::msg::PoseArray;
// using geometry_msgs::msg::PoseStamped;
// using geometry_msgs::msg::TransformStamped;
// using geometry_msgs::msg::Twist;
// using nav_msgs::msg::Odometry;
// using nav_msgs::msg::OccupancyGrid;


// PoseArray trajectory2PoseArray(const Trajectory & trajectory)
// {
//   PoseArray pose_array;
//   pose_array.header = trajectory.header;

//   for (const auto & point : trajectory.points) {
//     pose_array.poses.push_back(point.pose);
//   }

//   return pose_array;
// }

// std::vector<size_t> getReversingIndices(const Trajectory & trajectory)
// {
//   std::vector<size_t> indices;

//   for (size_t i = 0; i < trajectory.points.size() - 1; ++i) {
//     if (
//       trajectory.points.at(i).longitudinal_velocity_mps *
//         trajectory.points.at(i + 1).longitudinal_velocity_mps <
//       0) {
//       indices.push_back(i);
//     }
//   }

//   return indices;
// }

// size_t getNextTargetIndex(
//   const size_t trajectory_size, const std::vector<size_t> & reversing_indices,
//   const size_t current_target_index)
// {
//   if (!reversing_indices.empty()) {
//     for (const auto reversing_index : reversing_indices) {
//       if (reversing_index > current_target_index) {
//         return reversing_index;
//       }
//     }
//   }

//   return trajectory_size - 1;
// }

// Trajectory getPartialTrajectory(
//   const Trajectory & trajectory, const size_t start_index, const size_t end_index)
// {
//   Trajectory partial_trajectory;
//   partial_trajectory.header = trajectory.header;
//   partial_trajectory.header.stamp = rclcpp::Clock().now();

//   partial_trajectory.points.reserve(trajectory.points.size());
//   for (size_t i = start_index; i <= end_index; ++i) {
//     partial_trajectory.points.push_back(trajectory.points.at(i));
//   }

//   // Modify velocity at start/end point
//   if (partial_trajectory.points.size() >= 2) {
//     partial_trajectory.points.front().longitudinal_velocity_mps =
//       partial_trajectory.points.at(1).longitudinal_velocity_mps;
//   }
//   if (!partial_trajectory.points.empty()) {
//     partial_trajectory.points.back().longitudinal_velocity_mps = 0;
//   }

//   return partial_trajectory;
// }

// double calcDistance2d(const Trajectory & trajectory, const Pose & pose)
// {
//   const auto idx = motion_utils::findNearestIndex(trajectory.points, pose.position);
//   return tier4_autoware_utils::calcDistance2d(trajectory.points.at(idx), pose);
// }

// Pose transformPose(const Pose & pose, const TransformStamped & transform)
// {
//   PoseStamped transformed_pose;
//   PoseStamped orig_pose;
//   orig_pose.pose = pose;
//   tf2::doTransform(orig_pose, transformed_pose, transform);

//   return transformed_pose.pose;
// }

// Trajectory createTrajectory(
//   const PoseStamped & current_pose, const PlannerWaypoints & planner_waypoints,
//   const double & velocity)
// {
//   Trajectory trajectory;
//   trajectory.header = planner_waypoints.header;

//   for (const auto & awp : planner_waypoints.waypoints) {
//     TrajectoryPoint point;

//     point.pose = awp.pose.pose;

//     point.pose.position.z = current_pose.pose.position.z;  // height = const
//     point.longitudinal_velocity_mps = velocity / 3.6;      // velocity = const

//     // switch sign by forward/backward
//     point.longitudinal_velocity_mps = (awp.is_back ? -1 : 1) * point.longitudinal_velocity_mps;

//     trajectory.points.push_back(point);
//   }

//   return trajectory;
// }

// }  // namespace



// namespace obstacle_dstar
// {
// ObstacleDstarNode::ObstacleDstarNode(const rclcpp::NodeOptions & node_options)
// : Node("obstacle_dstar", node_options)
// {
//   using std::placeholders::_1;

//   // NodeParam
//   {
//     auto & p = node_param_;
//     p.planning_algorithm = declare_parameter<std::string>("planning_algorithm");
//     p.waypoints_velocity = declare_parameter<double>("waypoints_velocity");
//     p.update_rate = declare_parameter<double>("update_rate");
//     p.th_arrived_distance_m = declare_parameter<double>("th_arrived_distance_m");
//     p.th_stopped_time_sec = declare_parameter<double>("th_stopped_time_sec");
//     p.th_stopped_velocity_mps = declare_parameter<double>("th_stopped_velocity_mps");
//     p.th_course_out_distance_m = declare_parameter<double>("th_course_out_distance_m");
//     p.vehicle_shape_margin_m = declare_parameter<double>("vehicle_shape_margin_m");
//     p.replan_when_obstacle_found = declare_parameter<bool>("replan_when_obstacle_found");
//     p.replan_when_course_out = declare_parameter<bool>("replan_when_course_out");
    

//   }

//   // set vehicle_info
//   {
//     const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
//     vehicle_shape_.length = vehicle_info.vehicle_length_m;
//     vehicle_shape_.width = vehicle_info.vehicle_width_m;
//     vehicle_shape_.base2back = vehicle_info.rear_overhang_m;
//   }

//   // Planning
//   initializePlanningAlgorithm();

//   // Subscribers
//   {
//       trajectory_sub_ = create_subscription<Trajectory>(
//       "~/input/trajectory", rclcpp::QoS{1},
//       std::bind(&ObstacleDstarNode::onTrajectory, this, _1));
//     occupancy_grid_sub_ = create_subscription<OccupancyGrid>(
//       "~/input/occupancy_grid", 1, std::bind(&ObstacleDstarNode::onOccupancyGrid, this, _1));
//     odom_sub_ = create_subscription<Odometry>(
//       "~/input/odometry", 100, std::bind(&ObstacleDstarNode::onOdometry, this, _1));
//   }

//   // Publishers
//   {
//     rclcpp::QoS qos{1};
//     qos.transient_local();  // latch
//     trajectory_pub_ = create_publisher<Trajectory>("~/output/trajectory", qos);
//     occupancy_grid_pub_ = create_publisher<OccupancyGrid>("~/output/partial_grid", qos);
//   }

//   // TF
//   {
//     tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
//     tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
//   }

//   // Timer
//   {
//     const auto period_ns = rclcpp::Rate(node_param_.update_rate).period();
//     timer_ = rclcpp::create_timer(
//       this, get_clock(), period_ns, std::bind(&ObstacleDstarNode::onTimer, this));
//   }

//   logger_configure_ = std::make_unique<tier4_autoware_utils::LoggerLevelConfigure>(this);
//   is_detected = false;
// }

// PlannerCommonParam ObstacleDstarNode::getPlannerCommonParam()
// {
//   PlannerCommonParam p;

//   // search configs
//   p.time_limit = declare_parameter<double>("time_limit");
//   p.minimum_turning_radius = declare_parameter<double>("minimum_turning_radius");
//   p.maximum_turning_radius = declare_parameter<double>("maximum_turning_radius");
//   p.turning_radius_size = declare_parameter<int>("turning_radius_size");
//   p.maximum_turning_radius = std::max(p.maximum_turning_radius, p.minimum_turning_radius);
//   p.turning_radius_size = std::max(p.turning_radius_size, 1);

//   p.theta_size = declare_parameter<int>("theta_size");
//   p.angle_goal_range = declare_parameter<double>("angle_goal_range");
//   p.curve_weight = declare_parameter<double>("curve_weight");
//   p.reverse_weight = declare_parameter<double>("reverse_weight");
//   p.lateral_goal_range = declare_parameter<double>("lateral_goal_range");
//   p.longitudinal_goal_range = declare_parameter<double>("longitudinal_goal_range");

//   // costmap configs
//   p.obstacle_threshold = declare_parameter<int>("obstacle_threshold");

//   return p;
// }

// void ObstacleDstarNode::onTrajectory(
//   const Trajectory::ConstSharedPtr msg)
// {
//   trajectory_ptr = msg;
//   goal_pose_.header = msg->header;
  

//   // RCLCPP_INFO(get_logger(), "trajectory");

//   reset();
// }

// void ObstacleDstarNode::onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg)
// {
//   occupancy_grid_ = msg;
//   // std::string topic_name = rclcpp::get_topic_name(msg);
// //  std::string topic_name = occupancy_grid_sub_->get_topic_name();
// //  RCLCPP_INFO(get_logger(), 
// //               "Occupancy grid received on topic: %s, frame_id: %s", 
// //               topic_name.c_str(), 
// //               occupancy_grid_->header.frame_id.c_str());
// // logger_configure_->logTopics(get_topic_names_and_types());
// }

// void ObstacleDstarNode::onOdometry(const Odometry::ConstSharedPtr msg)
// {
//   odom_ = msg;
//   // RCLCPP_INFO(get_logger(), "odometry");

//   odom_buffer_.push_back(msg);

//   // Delete old data in buffer
//   while (true) {
//     const auto time_diff =
//       rclcpp::Time(msg->header.stamp) - rclcpp::Time(odom_buffer_.front()->header.stamp);

//     if (time_diff.seconds() < node_param_.th_stopped_time_sec) {
//       break;
//     }

//     odom_buffer_.pop_front();
//   }
// }

// bool ObstacleDstarNode::detectCollision()
//   {
//       if (partial_trajectory_.points.size() <= 1) {
//     RCLCPP_INFO(get_logger(), "no trajectory_points found");    
//     return true;
//   }

//     // size_t num_points = partial_trajectory_.points.size();
//     // RCLCPP_INFO(get_logger(), "points in trajectory: %zu", num_points);

//     for (const auto &point : partial_trajectory_.points)
//     {
//       // Check the surrounding area 5x5 on grid map for collision
//       const auto pose = point.pose;
//       int nearest_x = (pose.position.x - occupancy_grid_->info.origin.position.x )/occupancy_grid_->info.resolution;
//       int nearest_y = (pose.position.y - occupancy_grid_->info.origin.position.y )/occupancy_grid_->info.resolution;
    
//       // // if distance to the nearest index is greater than 0.2m, then the nearest index is not valid
//       // if (tier4_autoware_utils::calcDistance2d(partial_trajectory_.points.at(nearest_index), pose) > 0.2)
//       // {
//       //   RCLCPP_INFO(get_logger(), "nearest index too far" );
//       //   return false;
//       // }
//       // else
//       // {
//       //   grid_index = nearest_index;
//       // }
//       std::vector<int> surrounding_indices;
//       const int row_num = nearest_y;
//       const int col_num = nearest_x;

//       for (int i = -2; i <= 2; i++)
//       {
//         for (int j = -2; j <= 2; j++)
//         {
//           const int index = (row_num + i) * occupancy_grid_->info.width + col_num + j;
//           surrounding_indices.push_back(index);
//         }
//       }
      

//       for (const auto &index : surrounding_indices)
//       {
//         // RCLCPP_INFO(get_logger(), "index %i ", index);
//         // RCLCPP_INFO(get_logger(), "index value %i ", occupancy_grid_->data.at(index));
//         if (occupancy_grid_->data.at(index) > node_param_.obstacle_threshold)
//         {
//           return true;
//         }
//       }
//     }
//     RCLCPP_INFO(get_logger(), "done, but didn't find obstacle" );

//     return false;
// }

// bool ObstacleDstarNode::isPlanRequired()
// {
//   // RCLCPP_INFO(get_logger(), "is plan required");
//   if (trajectory_.points.size() <= 1) {
//     // RCLCPP_INFO(get_logger(), "Points empty");
//     return true;
//   }
  
//   // RCLCPP_INFO(get_logger(), "node param %d [s]", node_param_.replan_when_obstacle_found);

//   if (node_param_.replan_when_obstacle_found) {
//     algo_->setMap(*occupancy_grid_);
//     RCLCPP_INFO(get_logger(), "replan when obstacle");

//     const size_t nearest_index_partial =
//       motion_utils::findNearestIndex(partial_trajectory_.points, current_pose_.pose.position);
//     const size_t end_index_partial = partial_trajectory_.points.size() - 1;

//     const auto forward_trajectory =
//       getPartialTrajectory(partial_trajectory_, nearest_index_partial, end_index_partial);

//     const bool is_obstacle_found =
//       algo_->hasObstacleOnTrajectory(trajectory2PoseArray(forward_trajectory));
//     if (is_obstacle_found) {
//       RCLCPP_INFO(get_logger(), "Found obstacle");
//       return true;
//     }
//   }

//   if (node_param_.replan_when_course_out) {
//     const bool is_course_out =
//       calcDistance2d(trajectory_, current_pose_.pose) > node_param_.th_course_out_distance_m;
//     if (is_course_out) {
//       RCLCPP_INFO(get_logger(), "Course out");
//       return true;
//     }
//   }

//   return false;
// }

// void ObstacleDstarNode::updateTargetIndex()
// {
//   const auto is_near_target =
//     tier4_autoware_utils::calcDistance2d(trajectory_.points.at(target_index_), current_pose_) <
//     node_param_.th_arrived_distance_m;

//   if (is_near_target) {
//     const auto new_target_index =
//       getNextTargetIndex(partial_trajectory_.points.size(), reversing_indices_, target_index_);

//     if (new_target_index == target_index_) {
//     } else {
//       // Switch to next partial trajectory
//       prev_target_index_ = target_index_;
//       target_index_ =
//         getNextTargetIndex(partial_trajectory_.points.size(), reversing_indices_, target_index_);
//     }
//   }
// }

// void ObstacleDstarNode::getTrajectory(const Trajectory::ConstSharedPtr trajectory)
// {
//     partial_trajectory_.header = trajectory->header;
//     size_t nearest_index = motion_utils::findNearestIndex(trajectory->points, current_pose_.pose.position);
//     size_t goal_index = std::min(nearest_index + 12, trajectory->points.size() - 1);   // Add modulo operation to find max index at the end of the trajectory
//     partial_trajectory_.points.clear();

//     // Extract partial trajectory between nearest_index and goal_index
//     auto it = trajectory->points.begin();
//     std::advance(it, nearest_index); // Move iterator to nearest_index

//     for (size_t i = nearest_index; i <= goal_index; ++i)
//     {
//       partial_trajectory_.points.push_back(*it);
//       ++it;
//     }

//     std::advance(it, 30);

//     if (it != trajectory->points.end())
//     {
//       // Assign the pose at index nearest_index + 30 to goal_pose_.pose
//       goal_pose_.pose = it->pose;
//     }
  
// }

// void ObstacleDstarNode::onTimer()
// {
//   // Check all inputs are ready
//   if (!occupancy_grid_ || !trajectory_ptr || !odom_) {
//     // RCLCPP_INFO(get_logger(), "checking in timer");
//     return;
//   }
//   // RCLCPP_INFO(get_logger(), "lenght %f  width %f", vehicle_shape_.length, vehicle_shape_.width);

//   getTrajectory(trajectory_ptr);

//   // Get current pose
//   current_pose_.pose = odom_->pose.pose;
//   current_pose_.header = odom_->header;

//   if (current_pose_.header.frame_id == "") {
//     return;
//   }
//   extractPartialGridMap();
//   if(detectCollision())
//   {
//     getTrajectory(trajectory_ptr);
//     if (is_detected){
//       RCLCPP_INFO(get_logger(), "detected");

//       planTrajectory();
//     }
//     else {
//       is_detected = true;
//       algo_->setMap(*partial_grid_);
//       // algo_->clearNodes();
//         // RCLCPP_INFO(get_logger(), "false, not detected");
//       }
//   } else {
//     is_detected = false;
//   }
  

//   trajectory_pub_->publish(partial_trajectory_);

// }


// void ObstacleDstarNode::extractPartialGridMap()
// {
//   // Calculate the direction from currentPose to goalPose
//   double dx = goal_pose_.pose.position.x - current_pose_.pose.position.x;
//   double dy = goal_pose_.pose.position.y - current_pose_.pose.position.y;
//   double theta = std::atan2(dy, dx); // Direction angle

//   // Rectangle dimensions
//   double width = 200;
//   double height = 100;

//   // // Ensure the longer side is aligned with the direction vector
//   // if (std::abs(dx) < std::abs(dy)) {
//   //   std::swap(width, height);
//   // }

//   // Calculate the center of the rectangle
//   double centerX = (current_pose_.pose.position.x + goal_pose_.pose.position.x) / 2;
//   double centerY = (current_pose_.pose.position.y + goal_pose_.pose.position.y) / 2;

//   // Occupancy grid parameters
//   double resolution = occupancy_grid_->info.resolution;
//   double originX = occupancy_grid_->info.origin.position.x;
//   double originY = occupancy_grid_->info.origin.position.y;
//   int gridWidth = occupancy_grid_->info.width;
//   int gridHeight = occupancy_grid_->info.height;

//   // Create a new occupancy grid for the partial map
//   nav_msgs::msg::OccupancyGrid partial_grid_map;
//   partial_grid_map.header = occupancy_grid_->header;
//   partial_grid_map.info.resolution = resolution;
//   partial_grid_map.info.width = width;
//   partial_grid_map.info.height = height;
//   partial_grid_map.info.origin.position.x = centerX - (width * resolution) / 2;
//   partial_grid_map.info.origin.position.y = centerY - (height * resolution) / 2;
//   partial_grid_map.data.resize(width * height, -1);

//   // Rotate and translate the rectangle points to the occupancy grid coordinates
//   for (int i = 0; i < width; ++i) {
//     for (int j = 0; j < height; ++j) {
//       // Rectangle point in local coordinates
//       double localX = (i - width / 2) * resolution;
//       double localY = (j - height / 2) * resolution;

//       // Rotate point to align with the direction vector
//       double rotatedX = localX * std::cos(theta) - localY * std::sin(theta);
//       double rotatedY = localX * std::sin(theta) + localY * std::cos(theta);

//       // Translate to the center of the rectangle
//       double globalX = rotatedX + centerX;
//       double globalY = rotatedY + centerY;

//       // Convert to grid indices
//       int gridX = (globalX - originX) / resolution;
//       int gridY = (globalY - originY) / resolution;

//       // Check if indices are within bounds
//       if (gridX >= 0 && gridX < gridWidth && gridY >= 0 && gridY < gridHeight) {
//         int original_index = gridY * gridWidth + gridX;
//         int partial_index = j * width + i;
//         partial_grid_map.data[partial_index] = occupancy_grid_->data[original_index];
//       }
//     }
//   }

//   // Publish the partial grid map
//   partial_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(partial_grid_map);
//   occupancy_grid_pub_->publish(*partial_grid_);
// }


// void ObstacleDstarNode::planTrajectory()
// {
//   if (partial_grid_ == nullptr) {
//     return;
//   }
//   // Provide robot shape and map for the planner
//   algo_->setMap(*partial_grid_);

//   // Calculate poses in costmap frame
//   const auto current_pose_in_costmap_frame = transformPose(
//     current_pose_.pose,
//     getTransform(occupancy_grid_->header.frame_id, current_pose_.header.frame_id));

//   const auto goal_pose_in_costmap_frame = transformPose(
//     goal_pose_.pose, getTransform(occupancy_grid_->header.frame_id, goal_pose_.header.frame_id));
//   // RCLCPP_INFO(get_logger(), "current %f , goal %f ", current_pose_.pose.position.x, goal_pose_.pose.position.x);
  
//   // execute planning
//   const rclcpp::Time start = get_clock()->now();
//   const bool result = algo_->makePlan(current_pose_in_costmap_frame, goal_pose_in_costmap_frame);
//   const rclcpp::Time end = get_clock()->now();

//   if (result) {
//   RCLCPP_INFO(get_logger(), "Freespace planning: %f [s]", (end - start).seconds());
//     RCLCPP_INFO(get_logger(), "Found goal!");
//     trajectory_ =
//       createTrajectory(current_pose_, algo_->getWaypoints(), node_param_.waypoints_velocity);
//     size_t num_points = trajectory_.points.size();
//     RCLCPP_INFO(get_logger(), "points in trajectory: %zu", num_points);
    
//     reversing_indices_ = getReversingIndices(trajectory_);
//     prev_target_index_ = 0;
//     target_index_ =
//       getNextTargetIndex(trajectory_.points.size(), reversing_indices_, prev_target_index_);

//   } else {
//     // RCLCPP_INFO(get_logger(), "Can't find goal...");
//     reset();
//   }
// }

// void ObstacleDstarNode::reset()
// {
//   trajectory_ = Trajectory();
//   partial_trajectory_ = Trajectory();
// }

// TransformStamped ObstacleDstarNode::getTransform(
//   const std::string & from, const std::string & to)
// {
//   TransformStamped tf;
//   try {
//     tf =
//       tf_buffer_->lookupTransform(from, to, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
//   } catch (const tf2::TransformException & ex) {
//     RCLCPP_ERROR(get_logger(), "%s", ex.what());
//   }
//   return tf;
// }

// void ObstacleDstarNode::initializePlanningAlgorithm()
// {
//   // Extend robot shape
//   freespace_planning_algorithms::VehicleShape extended_vehicle_shape = vehicle_shape_;
//   const double margin = node_param_.vehicle_shape_margin_m;
//   extended_vehicle_shape.length += margin;
//   extended_vehicle_shape.width += margin;
//   extended_vehicle_shape.base2back += margin / 2;

//   const auto planner_common_param = getPlannerCommonParam();

//   const auto algo_name = node_param_.planning_algorithm;

//   // initialize specified algorithm
//   if (algo_name == "astar") {
//     algo_ = std::make_unique<AstarSearch>(planner_common_param, extended_vehicle_shape, *this);
//   } else if (algo_name == "rrtstar") {
//     algo_ = std::make_unique<RRTStar>(planner_common_param, extended_vehicle_shape, *this);
//   } else if (algo_name == "dstar")
//   {
//     algo_ = std::make_unique<Dstar>(planner_common_param, extended_vehicle_shape, *this);
//   }
//    else {
//     throw std::runtime_error("No such algorithm named " + algo_name + " exists.");
//   }
//   RCLCPP_INFO_STREAM(get_logger(), "initialize planning algorithm: " << algo_name);
// }
// }  // namespace obstacle_dstar

// #include "rclcpp_components/register_node_macro.hpp"

// RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_dstar::ObstacleDstarNode)
