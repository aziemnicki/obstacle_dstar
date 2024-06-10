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

#ifndef OBSTACLE_DSTAR__OBSTACLE_DSTAR_NODE_HPP_
#define OBSTACLE_DSTAR__OBSTACLE_DSTAR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "obstacle_dstar/obstacle_dstar.hpp"
#include "tier4_autoware_utils/ros/logger_level_configure.hpp"
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include "freespace_planning_algorithms/abstract_algorithm.hpp"

#include <freespace_planning_algorithms/dstar.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include "obstacle_velocity_limiter/occupancy_grid_utils.hpp"
#include <motion_utils/trajectory/trajectory.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <route_handler/route_handler.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/node.hpp>

#include <algorithm>
#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <typeinfo>



namespace obstacle_dstar
{
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using freespace_planning_algorithms::AbstractPlanningAlgorithm;
using freespace_planning_algorithms::PlannerCommonParam;
using freespace_planning_algorithms::DstarParam;
using freespace_planning_algorithms::DstarSearch;
using freespace_planning_algorithms::VehicleShape;
using freespace_planning_algorithms::PlannerWaypoint;
using freespace_planning_algorithms::PlannerWaypoints;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseArray;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Odometry;

struct NodeParam
{
  std::string planning_algorithm;
  double waypoints_velocity;  // constant velocity on planned waypoints [km/h]
  double update_rate;         // replanning and publishing rate [Hz]
  double th_arrived_distance_m;
  double th_stopped_time_sec;
  double th_stopped_velocity_mps;
  double th_course_out_distance_m;  // collision margin [m]
  double vehicle_shape_margin_m;
  int obstacle_threshold;
  
};


class ObstacleDstarNode : public rclcpp::Node
{
public:
  explicit ObstacleDstarNode(const rclcpp::NodeOptions & options);

private:
  // ros
  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr occupancy_grid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  rclcpp::Subscription<Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr occupancy_grid_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  // params
  NodeParam node_param_;
  VehicleShape vehicle_shape_;

  // variables
  std::unique_ptr<AbstractPlanningAlgorithm> algo_;
  PoseStamped current_pose_;
  PoseStamped goal_pose_;

  bool is_detected = false;

  Trajectory modified_trajectory_;
  Trajectory::ConstSharedPtr input_trajectory_;
  OccupancyGrid::ConstSharedPtr occupancy_grid_;
  OccupancyGrid::ConstSharedPtr partial_grid_;
  Odometry::ConstSharedPtr odom_;

  std::deque<Odometry::ConstSharedPtr> odom_buffer_;

  // functions used in the constructor
  PlannerCommonParam getPlannerCommonParam();

  // functions, callback
  void onTrajectory(const Trajectory::ConstSharedPtr msg);
  void onOccupancyGrid(const OccupancyGrid::ConstSharedPtr msg);
  void onOdometry(const Odometry::ConstSharedPtr msg);
  void onTimer();

  bool detectCollision();
  bool isPlanRequired();
  void extractPartialGridMap();
  void planTrajectory();

  void initializePlanningAlgorithm();

  std::unique_ptr<tier4_autoware_utils::LoggerLevelConfigure> logger_configure_;
};
}  // namespace obstacle_dstar

#endif  // OBSTACLE_DSTAR__OBSTACLE_DSTAR_NODE_HPP_

