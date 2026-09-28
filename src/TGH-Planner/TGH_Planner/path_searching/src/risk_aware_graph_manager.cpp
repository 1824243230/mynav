#include <path_searching/risk_aware_graph_manager.h>

#include <path_searching/topo_prm.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_set>

namespace fast_planner {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-9;
}

RiskAwareGraphManager::RiskAwareGraphManager(
    const std::shared_ptr<EDTEnvironment>& environment,
    const std::shared_ptr<RiskAwareEdge>& evaluator,
    std::list<NodePtr>& graph)
    : environment_(environment), evaluator_(evaluator), graph_(graph) {}

void RiskAwareGraphManager::init(ros::NodeHandle& nh) {
  nh.param("trg_management/enable", params_.enabled, params_.enabled);
  nh.param("trg_management/local_update_radius", params_.local_update_radius,
           params_.local_update_radius);
  nh.param("trg_management/expand_radius", params_.expand_radius,
           params_.expand_radius);
  nh.param("trg_management/merge_radius", params_.merge_radius,
           params_.merge_radius);
  nh.param("trg_management/connect_radius", params_.connect_radius,
           params_.connect_radius);
  nh.param("trg_management/node_risk_threshold", params_.node_risk_threshold,
           params_.node_risk_threshold);
  nh.param("trg_management/edge_risk_threshold", params_.edge_risk_threshold,
           params_.edge_risk_threshold);
  nh.param("trg_management/risk_change_threshold", params_.risk_change_threshold,
           params_.risk_change_threshold);
  nh.param("trg_management/switch_ratio", params_.switch_ratio,
           params_.switch_ratio);
  nh.param("trg_management/update_period", params_.update_period,
           params_.update_period);
  nh.param("trg_management/expansion_directions", params_.expansion_directions,
           params_.expansion_directions);
  nh.param("trg_management/max_frontiers_per_update",
           params_.max_frontiers_per_update, params_.max_frontiers_per_update);
  nh.param("trg_management/max_new_nodes_per_update",
           params_.max_new_nodes_per_update, params_.max_new_nodes_per_update);
  nh.param("trg_management/max_global_nodes", params_.max_global_nodes,
           params_.max_global_nodes);

  // 参数非法时回到兼容默认值；半径关系保证局部连接与空间桶索引有效。
  const Parameters defaults;
  auto positive = [](double value, double fallback) {
    return std::isfinite(value) && value > 0.0 ? value : fallback;
  };
  params_.local_update_radius = positive(params_.local_update_radius,
                                         defaults.local_update_radius);
  params_.expand_radius = positive(params_.expand_radius, defaults.expand_radius);
  params_.merge_radius = positive(params_.merge_radius, defaults.merge_radius);
  params_.connect_radius = std::max(params_.expand_radius,
      positive(params_.connect_radius, defaults.connect_radius));
  params_.node_risk_threshold = positive(params_.node_risk_threshold,
                                         defaults.node_risk_threshold);
  params_.edge_risk_threshold = positive(params_.edge_risk_threshold,
                                         defaults.edge_risk_threshold);
  params_.risk_change_threshold = positive(params_.risk_change_threshold,
                                            defaults.risk_change_threshold);
  params_.switch_ratio = std::isfinite(params_.switch_ratio)
      ? std::max(0.0, std::min(1.0, params_.switch_ratio)) : defaults.switch_ratio;
  params_.update_period = positive(params_.update_period, defaults.update_period);
  params_.expansion_directions = std::max(4, std::min(32, params_.expansion_directions));
  params_.max_frontiers_per_update = std::max(1, params_.max_frontiers_per_update);
  params_.max_new_nodes_per_update = std::max(1, params_.max_new_nodes_per_update);
  params_.max_global_nodes = std::max(2, params_.max_global_nodes);
  if (params_.enabled) {
    ROS_INFO_STREAM("[TRGManagement] enabled local_radius=" << params_.local_update_radius
                    << " expand_radius=" << params_.expand_radius
                    << " merge_radius=" << params_.merge_radius
                    << " connect_radius=" << params_.connect_radius
                    << " switch_ratio=" << params_.switch_ratio);
  }
}

RiskAwareGraphManager::BucketKey RiskAwareGraphManager::bucketKey(int x, int y) {
  return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(x)) << 32) |
         static_cast<std::uint32_t>(y);
}

RiskAwareGraphManager::BucketKey RiskAwareGraphManager::bucketOf(
    const Eigen::Vector3d& point) const {
  return bucketKey(static_cast<int>(std::floor(point.x() / params_.connect_radius)),
                   static_cast<int>(std::floor(point.y() / params_.connect_radius)));
}

std::vector<RiskAwareGraphManager::NodePtr> RiskAwareGraphManager::nearby(
    const Eigen::Vector3d& point, double radius) const {
  std::vector<NodePtr> nodes;
  if (!point.allFinite() || radius < 0.0) return nodes;
  const int min_x = static_cast<int>(std::floor((point.x() - radius) /
                                                 params_.connect_radius));
  const int max_x = static_cast<int>(std::floor((point.x() + radius) /
                                                 params_.connect_radius));
  const int min_y = static_cast<int>(std::floor((point.y() - radius) /
                                                 params_.connect_radius));
  const int max_y = static_cast<int>(std::floor((point.y() + radius) /
                                                 params_.connect_radius));
  const double radius_sq = radius * radius;
  for (int x = min_x; x <= max_x; ++x) {
    for (int y = min_y; y <= max_y; ++y) {
      const auto bucket = buckets_.find(bucketKey(x, y));
      if (bucket == buckets_.end()) continue;
      for (const auto& node : bucket->second) {
        if ((node->pos_ - point).head<2>().squaredNorm() <= radius_sq)
          nodes.push_back(node);
      }
    }
  }
  return nodes;
}

RiskAwareGraphManager::NodePtr RiskAwareGraphManager::nearest(
    const Eigen::Vector3d& point, double radius) const {
  NodePtr best;
  double best_distance = radius * radius;
  for (const auto& node : nearby(point, radius)) {
    const double distance = (node->pos_ - point).head<2>().squaredNorm();
    if (distance <= best_distance) {
      best = node;
      best_distance = distance;
    }
  }
  return best;
}

RiskAwareGraphManager::NodePtr RiskAwareGraphManager::addNode(
    const Eigen::Vector3d& point) {
  if (graph_.size() >= static_cast<size_t>(params_.max_global_nodes)) return {};
  auto node = std::make_shared<GraphNode>(point, GraphNode::Guard, next_node_id_++);
  graph_.push_back(node);
  buckets_[bucketOf(point)].push_back(node);
  return node;
}

bool RiskAwareGraphManager::nodeValid(const Eigen::Vector3d& point) const {
  if (!point.allFinite() || !environment_ || !environment_->sdf_map_) return false;
  const Eigen::Vector2d xy = point.head<2>();
  auto map = environment_->sdf_map_;
  const auto risk_map = map->getRiskMapManager();
  if (!map->isInMap2D(xy) || map->getInflateOccupancy2D(xy) == 1 ||
      !risk_map || !risk_map->isReady()) return false;
  const double risk = risk_map->getRisk(xy);
  return std::isfinite(risk) && risk <= params_.node_risk_threshold;
}

bool RiskAwareGraphManager::edgeValid(const Eigen::Vector3d& from,
                                      const Eigen::Vector3d& to,
                                      double& risk) const {
  risk = std::numeric_limits<double>::infinity();
  if (!nodeValid(from) || !nodeValid(to) || !evaluator_) return false;
  const double length = (to - from).head<2>().norm();
  if (length < 1e-6) return false;
  const auto map = environment_->sdf_map_;
  const double step = std::max(0.02, map->getResolution());
  const int steps = std::max(1, static_cast<int>(std::ceil(length / step)));
  for (int index = 0; index <= steps; ++index) {
    const double fraction = static_cast<double>(index) / steps;
    const Eigen::Vector2d xy = ((1.0 - fraction) * from + fraction * to).head<2>();
    if (!map->isInMap2D(xy) || map->getInflateOccupancy2D(xy) == 1)
      return false;
  }
  // 复用 RiskAwareEdge 的采样/CVaR 与最大单点风险约束。
  const RiskEdge edge = evaluator_->evaluateEdge(0, 1, from, to);
  risk = edge.risk_cost;
  return std::isfinite(risk) && std::isfinite(edge.maximum_risk) &&
         edge.maximum_risk <= params_.edge_risk_threshold;
}

bool RiskAwareGraphManager::connect(const NodePtr& from, const NodePtr& to) {
  if (!from || !to || from == to || !from->valid_ || !to->valid_) return false;
  if ((from->pos_ - to->pos_).head<2>().norm() > params_.connect_radius)
    return false;
  for (size_t index = 0; index < from->neighbors_.size(); ++index) {
    if (from->neighbors_[index] != to) continue;
    if (index < from->managed_edges_.size() && from->managed_edges_[index].valid)
      return true;
    // 地图更新后，历史上失效的边仍可重新恢复；双向状态必须同步。
    double restored_risk = 0.0;
    if (!edgeValid(from->pos_, to->pos_, restored_risk)) return false;
    const ManagedTopoEdge restored{true, restored_risk,
                                   ros::WallTime::now().toSec()};
    if (index < from->managed_edges_.size())
      from->managed_edges_[index] = restored;
    for (size_t reverse = 0; reverse < to->neighbors_.size(); ++reverse) {
      if (to->neighbors_[reverse] == from &&
          reverse < to->managed_edges_.size()) {
        to->managed_edges_[reverse] = restored;
        break;
      }
    }
    return true;
  }
  double risk = 0.0;
  if (!edgeValid(from->pos_, to->pos_, risk)) return false;
  const double stamp = ros::WallTime::now().toSec();
  from->neighbors_.push_back(to);
  from->managed_edges_.push_back({true, risk, stamp});
  to->neighbors_.push_back(from);
  to->managed_edges_.push_back({true, risk, stamp});
  ++edge_count_;
  return true;
}

void RiskAwareGraphManager::seedFromPaths(
    const std::vector<std::vector<Eigen::Vector3d>>& paths) {
  if (!params_.enabled) return;
  const double spacing = std::min(params_.expand_radius,
                                  0.5 * params_.connect_radius);
  for (const auto& path : paths) {
    NodePtr previous;
    for (size_t segment = 0; segment < path.size(); ++segment) {
      const int steps = segment == 0 ? 0 : std::max(1, static_cast<int>(std::ceil(
          (path[segment] - path[segment - 1]).head<2>().norm() / spacing)));
      for (int step = segment == 0 ? 0 : 1; step <= steps; ++step) {
        Eigen::Vector3d point = path.front();
        if (segment > 0) {
          const double fraction = static_cast<double>(step) / steps;
          point = (1.0 - fraction) * path[segment - 1] +
                  fraction * path[segment];
        }
        if (!nodeValid(point)) {
          previous.reset();
          continue;
        }
        NodePtr current = nearest(point, params_.merge_radius);
        if (current && !nodeValid(current->pos_)) {
          current->valid_ = false;
          current.reset();
        }
        if (!current) current = addNode(point);
        if (!current) return;
        current->valid_ = true;  // 新点或复用点已经过当前地图检查。
        if (previous && previous != current && !connect(previous, current)) {
          // 边不可通行时停止延续该路径，避免把断开的几何段误记为可达。
          previous = current;
          continue;
        }
        previous = current;
      }
    }
  }
}

std::vector<Eigen::Vector3d> RiskAwareGraphManager::findManagedPath(
    const Eigen::Vector3d& start, const Eigen::Vector3d& goal) const {
  std::vector<Eigen::Vector3d> path;
  if (!params_.enabled || graph_.empty() || !start.allFinite() || !goal.allFinite())
    return path;
  struct QueueItem { double cost; NodePtr node; };
  const auto greater = [](const QueueItem& a, const QueueItem& b) {
    return a.cost > b.cost;
  };
  std::priority_queue<QueueItem, std::vector<QueueItem>, decltype(greater)> open(greater);
  std::unordered_map<int, double> distance;
  std::unordered_map<int, NodePtr> predecessor;
  double risk = 0.0;
  for (const auto& node : nearby(start, params_.connect_radius)) {
    if (!node->valid_) continue;
    const double length = (node->pos_ - start).head<2>().norm();
    if (length > 1e-6 && !edgeValid(start, node->pos_, risk)) continue;
    const double cost = length > 1e-6 ? length + risk : 0.0;
    distance[node->id_] = cost;
    open.push({cost, node});
  }
  NodePtr reached;
  while (!open.empty()) {
    const QueueItem item = open.top();
    open.pop();
    if (item.cost > distance[item.node->id_] + kEpsilon) continue;
    const double goal_distance = (item.node->pos_ - goal).head<2>().norm();
    if (goal_distance <= params_.connect_radius &&
        (goal_distance < 1e-6 || edgeValid(item.node->pos_, goal, risk))) {
      reached = item.node;
      break;
    }
    const auto& neighbors = item.node->neighbors_;
    for (size_t i = 0; i < neighbors.size(); ++i) {
      const auto& next = neighbors[i];
      if (!next || !next->valid_ || i >= item.node->managed_edges_.size() ||
          !item.node->managed_edges_[i].valid) continue;
      const double edge_length = (item.node->pos_ - next->pos_).head<2>().norm();
      const double candidate = item.cost + edge_length +
                               item.node->managed_edges_[i].risk;
      const auto old = distance.find(next->id_);
      if (old != distance.end() && candidate >= old->second) continue;
      distance[next->id_] = candidate;
      predecessor[next->id_] = item.node;
      open.push({candidate, next});
    }
  }
  if (!reached) return path;
  if ((reached->pos_ - goal).head<2>().norm() > 1e-6) path.push_back(goal);
  for (NodePtr node = reached; node; ) {
    path.push_back(node->pos_);
    const auto parent = predecessor.find(node->id_);
    node = parent == predecessor.end() ? NodePtr() : parent->second;
  }
  if ((path.back() - start).head<2>().norm() > 1e-6) path.push_back(start);
  std::reverse(path.begin(), path.end());
  return path;
}

void RiskAwareGraphManager::setCurrentPath(
    const std::vector<Eigen::Vector3d>& path) {
  if (!params_.enabled) return;
  current_path_ = path;
  current_path_baseline_risk_.clear();
  current_path_baseline_risk_.reserve(path.size());
  const auto risk_map = environment_->sdf_map_->getRiskMapManager();
  for (const auto& point : path) {
    const double risk = risk_map && risk_map->isReady()
        ? risk_map->getRisk(point.head<2>()) : 0.0;
    current_path_baseline_risk_.push_back(risk);
  }
}

std::vector<RiskAwareGraphManager::NodePtr>
RiskAwareGraphManager::extractLocalNodes(const Eigen::Vector3d& pose) const {
  return nearby(pose, params_.local_update_radius);
}

size_t RiskAwareGraphManager::updateNodeValidity(
    const std::vector<NodePtr>& nodes, double stamp) {
  size_t invalid_count = 0;
  for (const auto& node : nodes) {
    node->valid_ = nodeValid(node->pos_);
    node->last_update_ = stamp;
    if (!node->valid_) ++invalid_count;
  }
  return invalid_count;
}

size_t RiskAwareGraphManager::updateLocalEdgeRisk(
    const std::vector<NodePtr>& nodes, double stamp) {
  std::unordered_set<int> local_ids;
  local_ids.reserve(nodes.size());
  for (const auto& node : nodes) local_ids.insert(node->id_);
  size_t invalid_count = 0;
  for (const auto& node : nodes) {
    for (size_t i = 0; i < node->neighbors_.size(); ++i) {
      const auto& neighbor = node->neighbors_[i];
      if (!neighbor || node->id_ >= neighbor->id_ ||
          !local_ids.count(neighbor->id_) || i >= node->managed_edges_.size()) continue;
      double risk = std::numeric_limits<double>::infinity();
      const bool valid = node->valid_ && neighbor->valid_ &&
          edgeValid(node->pos_, neighbor->pos_, risk);
      node->managed_edges_[i] = {valid, risk, stamp};
      for (size_t reverse = 0; reverse < neighbor->neighbors_.size(); ++reverse) {
        if (neighbor->neighbors_[reverse] == node &&
            reverse < neighbor->managed_edges_.size()) {
          neighbor->managed_edges_[reverse] = {valid, risk, stamp};
          break;
        }
      }
      if (!valid) ++invalid_count;
    }
  }
  return invalid_count;
}

std::vector<RiskAwareGraphManager::NodePtr>
RiskAwareGraphManager::detectFrontiers(const std::vector<NodePtr>& nodes,
                                      const Eigen::Vector3d& pose) {
  std::vector<NodePtr> frontiers;
  const auto risk_map = environment_->sdf_map_->getRiskMapManager();
  // robot_width 为车体直径近似，等于论文中建议的 2*r_robot。
  const double probe_distance = risk_map
      ? std::max(params_.merge_radius, risk_map->getParameters().robot_width)
      : 2.0 * params_.merge_radius;
  for (const auto& node : nodes) {
    node->frontier_ = false;
    if (!node->valid_) continue;
    const Eigen::Vector2d direction = (node->pos_ - pose).head<2>();
    if (direction.norm() < 1e-6) continue;
    Eigen::Vector3d probe = node->pos_;
    probe.head<2>() += probe_distance * direction.normalized();
    // 不扫描整张图；只查 probe 所落的少数空间桶。
    if (!nearest(probe, params_.merge_radius)) {
      node->frontier_ = true;
      frontiers.push_back(node);
    }
  }
  return frontiers;
}

size_t RiskAwareGraphManager::expandFrontiers(
    const std::vector<NodePtr>& frontiers, const Eigen::Vector3d& pose) {
  size_t added = 0;
  int visited = 0;
  for (const auto& frontier : frontiers) {
    if (++visited > params_.max_frontiers_per_update ||
        added >= static_cast<size_t>(params_.max_new_nodes_per_update)) break;
    for (int direction = 0; direction < params_.expansion_directions; ++direction) {
      if (added >= static_cast<size_t>(params_.max_new_nodes_per_update) ||
          graph_.size() >= static_cast<size_t>(params_.max_global_nodes)) break;
      const double angle = 2.0 * kPi * direction / params_.expansion_directions;
      Eigen::Vector3d point = frontier->pos_;
      point.x() += params_.expand_radius * std::cos(angle);
      point.y() += params_.expand_radius * std::sin(angle);
      if ((point - pose).head<2>().norm() >
          params_.local_update_radius + params_.expand_radius || !nodeValid(point)) continue;
      NodePtr neighbor = nearest(point, params_.merge_radius);
      if (neighbor) {
        neighbor->valid_ = nodeValid(neighbor->pos_);
        if (neighbor->valid_) {
          connect(frontier, neighbor);
          continue;
        }
      }
      double risk = 0.0;
      if (!edgeValid(frontier->pos_, point, risk)) continue;
      NodePtr created = addNode(point);
      if (!created) break;
      connect(frontier, created);
      // 仅连接新点附近节点，复用同一边碰撞/风险评价。
      for (const auto& candidate : nearby(point, params_.connect_radius))
        if (candidate != created && candidate != frontier && candidate->valid_)
          connect(created, candidate);
      ++added;
    }
  }
  return added;
}

bool RiskAwareGraphManager::checkCurrentPathValid(
    const Eigen::Vector3d& pose) const {
  if (current_path_.size() < 2 ||
      current_path_baseline_risk_.size() != current_path_.size()) return false;
  size_t nearest_index = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < current_path_.size(); ++i) {
    const double distance = (current_path_[i] - pose).head<2>().squaredNorm();
    if (distance < nearest_distance) {
      nearest_distance = distance;
      nearest_index = i;
    }
  }
  const auto risk_map = environment_->sdf_map_->getRiskMapManager();
  double current_sum = 0.0;
  double baseline_sum = 0.0;
  size_t count = 0;
  for (size_t i = nearest_index; i < current_path_.size(); ++i) {
    const auto& point = current_path_[i];
    if ((point - pose).head<2>().norm() > params_.local_update_radius) break;
    if (!nodeValid(point)) return true;
    const double current_risk = risk_map->getRisk(point.head<2>());
    if (!std::isfinite(current_risk) ||
        current_risk > params_.edge_risk_threshold) return true;
    current_sum += current_risk;
    baseline_sum += current_path_baseline_risk_[i];
    ++count;
    if (i + 1 < current_path_.size() &&
        (current_path_[i + 1] - pose).head<2>().norm() <=
            params_.local_update_radius) {
      double risk = 0.0;
      if (!edgeValid(point, current_path_[i + 1], risk)) return true;
    }
  }
  if (count == 0 || !std::isfinite(baseline_sum)) return false;
  const double baseline = baseline_sum / count;
  const double current = current_sum / count;
  // 基线风险接近零时，微小的地图数值波动不应放大为一次重规划。
  return (current - baseline) / std::max(1.0, baseline) >
         params_.risk_change_threshold;
}

bool RiskAwareGraphManager::updateGraphManagement(
    const Eigen::Vector3d& robot_pose) {
  if (!params_.enabled || !robot_pose.allFinite()) return false;
  const ros::WallTime now = ros::WallTime::now();
  if (last_update_wall_.toSec() > 0.0 &&
      (now - last_update_wall_).toSec() < params_.update_period) return false;
  last_update_wall_ = now;
  const auto risk_map = environment_->sdf_map_->getRiskMapManager();
  if (!risk_map || !risk_map->isReady()) return false;
  const auto local = extractLocalNodes(robot_pose);
  const size_t invalid_nodes = updateNodeValidity(local, now.toSec());
  const size_t invalid_edges = updateLocalEdgeRisk(local, now.toSec());
  const auto frontiers = detectFrontiers(local, robot_pose);
  const size_t added = expandFrontiers(frontiers, robot_pose);
  const bool need_replan = checkCurrentPathValid(robot_pose);
  ROS_INFO_STREAM("[TRGManagement] global_nodes=" << graph_.size()
                  << " global_edges=" << edge_count_
                  << " local_nodes=" << local.size()
                  << " frontiers=" << frontiers.size()
                  << " new_nodes=" << added
                  << " invalid_nodes=" << invalid_nodes
                  << " invalid_edges=" << invalid_edges
                  << " need_replan=" << std::boolalpha << need_replan);
  return need_replan;
}

}  // namespace fast_planner
