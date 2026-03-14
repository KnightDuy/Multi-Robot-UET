
#include <string>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>

#include "simple_astar_planner.hpp"
#include <algorithm>
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace nav2_simple_astar_planner {

struct pair_hash {
  template <class T1, class T2>
  std::size_t operator()(const std::pair<T1, T2>& p) const {
    auto h1 = std::hash<T1>{}(p.first);
    auto h2 = std::hash<T2>{}(p.second);
    return h1 ^ (h2 << 1);
  }
};

std::vector<std::pair<int, int>> nav2_simple_astar_planner::SimpleAStarPlanner::a_star_search(
  int start_x, int start_y, int goal_x, int goal_y) {
  using Node = std::pair<int, int>;
  auto heuristic = [goal_x, goal_y](int x, int y) {
    // Euclidean distance
    return std::hypot(goal_x - x, goal_y - y);
  };

  std::priority_queue<
    std::tuple<double, int, int>,
    std::vector<std::tuple<double, int, int>>,
    std::greater<>> open;
  std::unordered_map<Node, Node, pair_hash> came_from;
  std::unordered_map<Node, double, pair_hash> g_score;
  std::unordered_set<Node, pair_hash> closed;

  open.emplace(heuristic(start_x, start_y), start_x, start_y);
  g_score[{start_x, start_y}] = 0.0;

  int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  int w = costmap_->getSizeInCellsX();
  int h = costmap_->getSizeInCellsY();

  while (!open.empty()) {
    auto [f, x, y] = open.top();
    open.pop();
    Node current = {x, y};
    if (closed.count(current)) continue;
    closed.insert(current);
    if (x == goal_x && y == goal_y) {
      // Reconstruct path
      std::vector<Node> path;
      Node n = current;
      while (came_from.count(n)) {
        path.push_back(n);
        n = came_from[n];
      }
      path.push_back({start_x, start_y});
      return path;
    }
    for (int dir = 0; dir < 8; ++dir) {
      int nx = x + dx[dir];
      int ny = y + dy[dir];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      unsigned char cost = costmap_->getCost(nx, ny);
      // Avoid cells too close to obstacles (e.g., within 5cm)
      // Assume costmap resolution is in meters, so 5cm = 0.05m
      // Use inflation: skip cells with cost above a threshold (e.g., 253 for nav2)
      unsigned char min_safe_cost = 253; // adjust as needed for your inflation layer
      if (cost >= min_safe_cost) continue;
      Node neighbor = {nx, ny};
      double move_cost = (dir % 2 == 0) ? 1.0 : std::sqrt(2.0); 
      double tentative_g = g_score[current] + move_cost + cost / 255.0;
      if (!g_score.count(neighbor) || tentative_g < g_score[neighbor]) {
        g_score[neighbor] = tentative_g;
        double f = tentative_g + heuristic(nx, ny);
        open.emplace(f, nx, ny);
        came_from[neighbor] = current;
      }
    }
  }
  return {};
}

void nav2_simple_astar_planner::SimpleAStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer>,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  name_ = name;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  // Try to get the local costmap if available (for hybrid planning)
  // In standard Nav2, if this planner is loaded in the local planner server, costmap_ros is already the local costmap
  local_costmap_ = costmap_ros->getCostmap();

  RCLCPP_INFO(logger_, "Configuring SimpleAStarPlanner plugin: %s", name_.c_str());

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".tolerance", rclcpp::ParameterValue(0.5));
  node->get_parameter(name_ + ".tolerance", tolerance_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_iterations", rclcpp::ParameterValue(1000000));
  node->get_parameter(name_ + ".max_iterations", max_iterations_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_on_approach_iterations", rclcpp::ParameterValue(1000));
  node->get_parameter(name_ + ".max_on_approach_iterations", max_on_approach_iterations_);

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".max_planning_time", rclcpp::ParameterValue(5.0));
  node->get_parameter(name_ + ".max_planning_time", max_planning_time_);

  RCLCPP_INFO(logger_, "SimpleAStarPlanner configured successfully");
}

void nav2_simple_astar_planner::SimpleAStarPlanner::activate()
{
  RCLCPP_INFO(logger_, "Activating SimpleAStarPlanner");
}

void nav2_simple_astar_planner::SimpleAStarPlanner::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating SimpleAStarPlanner");
}

void nav2_simple_astar_planner::SimpleAStarPlanner::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up SimpleAStarPlanner");
}

nav_msgs::msg::Path nav2_simple_astar_planner::SimpleAStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap_->getMutex()));
  (void)cancel_checker;
  unsigned int mx_start, my_start, mx_goal, my_goal;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start)) {
    throw std::runtime_error("Start position is outside map bounds");
  }
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)) {
    throw std::runtime_error("Goal position is outside map bounds");
  }
  if (mx_start == mx_goal && my_start == my_goal) {
    nav_msgs::msg::Path path;
    path.header.stamp = clock_->now();
    path.header.frame_id = global_frame_;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose = start.pose;
    path.poses.push_back(pose);
    return path;
  }

  // 1. Plan global path using global costmap
  std::vector<std::pair<int, int>> global_path = this->a_star_search(mx_start, my_start, mx_goal, my_goal);
  if (global_path.empty()) {
    throw std::runtime_error("No valid global path found");
  }

  // 2. Check for new obstacles along the global path using local costmap
  // (Assume local_costmap_ is set up and points to the local costmap)
  // If not, fallback to global only
  nav2_costmap_2d::Costmap2D* local_costmap = local_costmap_ ? local_costmap_ : costmap_;

  // 3. Find the first collision on the global path in the local costmap
  size_t detour_start_idx = 0;
  for (size_t i = 0; i < global_path.size(); ++i) {
    unsigned char cost = local_costmap->getCost(global_path[i].first, global_path[i].second);
    if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      detour_start_idx = i;
      break;
    }
  }

  nav_msgs::msg::Path path;
  path.header.stamp = clock_->now();
  path.header.frame_id = global_frame_;
  geometry_msgs::msg::PoseStamped pose;
  pose.header = path.header;

  // 4. If no collision, use global path
  if (detour_start_idx == 0) {
    for (int i = global_path.size() - 1; i >= 0; --i) {
      double wx, wy;
      costmap_->mapToWorld(
        static_cast<unsigned int>(global_path[i].first),
        static_cast<unsigned int>(global_path[i].second),
        wx, wy);
      pose.pose.position.x = wx;
      pose.pose.position.y = wy;
      pose.pose.position.z = 0.0;
      if (i > 0) {
        double dx = global_path[i].first - global_path[i-1].first;
        double dy = global_path[i].second - global_path[i-1].second;
        double theta = atan2(dy, dx);
        pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);
      }
      path.poses.push_back(pose);
    }
    geometry_msgs::msg::PoseStamped goal_pose = goal;
    goal_pose.header = path.header;
    path.poses.push_back(goal_pose);
    RCLCPP_INFO(logger_, "Path created with %zu poses (global only)", path.poses.size());
    return path;
  }

  // 5. If collision, plan a local detour from detour_start_idx-1 to a safe point after the obstacle
  size_t detour_end_idx = detour_start_idx;
  for (size_t i = detour_start_idx; i < global_path.size(); ++i) {
    unsigned char cost = local_costmap->getCost(global_path[i].first, global_path[i].second);
    if (cost < nav2_costmap_2d::LETHAL_OBSTACLE) {
      detour_end_idx = i;
      break;
    }
  }
  // Plan local detour
  int detour_start_x = global_path[detour_start_idx-1].first;
  int detour_start_y = global_path[detour_start_idx-1].second;
  int detour_end_x = global_path[detour_end_idx].first;
  int detour_end_y = global_path[detour_end_idx].second;
  std::vector<std::pair<int, int>> local_detour = this->a_star_search(detour_start_x, detour_start_y, detour_end_x, detour_end_y);
  // If local detour fails (local minima), do improved wall-following with progress check, goal bias, and random search
  if (local_detour.empty()) {
    RCLCPP_WARN(logger_, "Local detour failed, attempting improved wall-following");
    int wx = detour_start_x;
    int wy = detour_start_y;
    int w = local_costmap->getSizeInCellsX();
    int h = local_costmap->getSizeInCellsY();
    int dx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    int dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
    std::vector<std::pair<int, int>> wall_path;
    wall_path.push_back({wx, wy});
    bool wall_escaped = false;
    double min_dist_to_goal = std::hypot((double)wx - detour_end_x, (double)wy - detour_end_y);
    int max_steps = 20;
    for (int step = 0; step < max_steps; ++step) {
      bool found = false;
      // 2. Goal bias: try to leave wall-following if a direction toward the goal is open
      double best_goal_dist = min_dist_to_goal;
      int best_nx = wx, best_ny = wy;
      for (int dir = 0; dir < 8; ++dir) {
        int nx = wx + dx[dir];
        int ny = wy + dy[dir];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        unsigned char cost = local_costmap->getCost(nx, ny);
        if (cost < nav2_costmap_2d::LETHAL_OBSTACLE) {
          double dist_to_goal = std::hypot((double)nx - detour_end_x, (double)ny - detour_end_y);
          if (dist_to_goal < best_goal_dist) {
            wx = nx;
            wy = ny;
            wall_path.push_back({wx, wy});
            min_dist_to_goal = dist_to_goal;
            wall_escaped = true;
            found = true;
            break;
          }
        }
      }
      if (wall_escaped) break;
      // 1. Progress check: only follow wall if not making progress, else break
      double prev_dist = min_dist_to_goal;
      for (int dir = 0; dir < 8; ++dir) {
        int nx = wx + dx[dir];
        int ny = wy + dy[dir];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        unsigned char cost = local_costmap->getCost(nx, ny);
        if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
          int next_dir = (dir + 2) % 8;
          int tx = wx + dx[next_dir];
          int ty = wy + dy[next_dir];
          if (tx >= 0 && ty >= 0 && tx < w && ty < h && local_costmap->getCost(tx, ty) < nav2_costmap_2d::LETHAL_OBSTACLE) {
            wx = tx;
            wy = ty;
            wall_path.push_back({wx, wy});
            found = true;
            double dist_to_goal = std::hypot((double)wx - detour_end_x, (double)wy - detour_end_y);
            if (dist_to_goal < min_dist_to_goal) {
              min_dist_to_goal = dist_to_goal;
            }
            break;
          }
        }
      }
      // 3. Randomized/spiral search if stuck
      if (!found && step == max_steps/2) {
        for (int spiral = 1; spiral <= 2; ++spiral) {
          for (int dir = 0; dir < 8; ++dir) {
            int nx = wx + spiral * dx[dir];
            int ny = wy + spiral * dy[dir];
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            unsigned char cost = local_costmap->getCost(nx, ny);
            if (cost < nav2_costmap_2d::LETHAL_OBSTACLE) {
              wx = nx;
              wy = ny;
              wall_path.push_back({wx, wy});
              found = true;
              min_dist_to_goal = std::hypot((double)wx - detour_end_x, (double)wy - detour_end_y);
              break;
            }
          }
          if (found) break;
        }
      }
      if (!found) break;
    }
    // 4. Aggressive recovery: only accept wall_path if it gets closer to the goal
    double final_dist = std::hypot((double)wx - detour_end_x, (double)wy - detour_end_y);
    if (wall_escaped && final_dist < std::hypot((double)detour_start_x - detour_end_x, (double)detour_start_y - detour_end_y)) {
      local_detour = wall_path;
    } else {
      RCLCPP_ERROR(logger_, "Improved wall-following failed, returning no path for recovery");
      throw std::runtime_error("No valid path found (local minimum)");
    }
  }

  // 6. Build the final path: global up to detour, local detour, then global to goal
  for (int i = global_path.size() - 1; i >= (int)detour_end_idx; --i) {
    double wx, wy;
    costmap_->mapToWorld(
      static_cast<unsigned int>(global_path[i].first),
      static_cast<unsigned int>(global_path[i].second),
      wx, wy);
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    if (i > 0) {
      double dx = global_path[i].first - global_path[i-1].first;
      double dy = global_path[i].second - global_path[i-1].second;
      double theta = atan2(dy, dx);
      pose.pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);
    }
    path.poses.push_back(pose);
  }
  for (int i = local_detour.size() - 1; i >= 0; --i) {
    double wx, wy;
    costmap_->mapToWorld(
      static_cast<unsigned int>(local_detour[i].first),
      static_cast<unsigned int>(local_detour[i].second),
      wx, wy);
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    path.poses.push_back(pose);
  }
  for (int i = detour_start_idx - 1; i >= 0; --i) {
    double wx, wy;
    costmap_->mapToWorld(
      static_cast<unsigned int>(global_path[i].first),
      static_cast<unsigned int>(global_path[i].second),
      wx, wy);
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    pose.pose.position.z = 0.0;
    path.poses.push_back(pose);
  }
  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header = path.header;
  path.poses.push_back(goal_pose);
  RCLCPP_INFO(logger_, "Path created with %zu poses (hybrid global-local)", path.poses.size());
  return path;
}

} // namespace nav2_simple_astar_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_simple_astar_planner::SimpleAStarPlanner, nav2_core::GlobalPlanner)
