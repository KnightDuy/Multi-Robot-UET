#pragma once

#include <string>
#include <memory>
#include <functional>
#include <vector>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2_simple_astar_planner
{

class SimpleAStarPlanner : public nav2_core::GlobalPlanner
{
public:
  SimpleAStarPlanner() = default;
  ~SimpleAStarPlanner() = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

protected:
  nav2_costmap_2d::Costmap2D * costmap_;
  nav2_costmap_2d::Costmap2D * local_costmap_ = nullptr;
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string global_frame_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("SimpleAStarPlanner")};
  rclcpp::Clock::SharedPtr clock_;
  double tolerance_;
  int max_iterations_;
  int max_on_approach_iterations_;
  bool allow_unknown_;
  double max_planning_time_;
  unsigned char max_allowed_cost_;

public:
  void setLocalCostmap(nav2_costmap_2d::Costmap2D * local_costmap) { local_costmap_ = local_costmap; }

public:
  std::vector<std::pair<int, int>> a_star_search(int start_x, int start_y, int goal_x, int goal_y);
};

}
