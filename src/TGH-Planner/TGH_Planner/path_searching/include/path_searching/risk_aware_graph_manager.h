#pragma once

#include <Eigen/Core>
#include <ros/ros.h>

#include <cstdint>
#include <list>
#include <memory>
#include <unordered_map>
#include <vector>

namespace fast_planner {

class EDTEnvironment;
class GraphNode;
class RiskAwareEdge;

// 管理 TopologyPRM 已有的 graph_，不替换 Voronoi/TopoPRM 搜索器。
class RiskAwareGraphManager {
 public:
  struct Parameters {
    bool enabled = false;
    double local_update_radius = 4.0;
    double expand_radius = 1.0;
    double merge_radius = 0.35;
    double connect_radius = 2.0;
    double node_risk_threshold = 150.0;
    double edge_risk_threshold = 150.0;
    double risk_change_threshold = 0.25;
    double switch_ratio = 0.15;
    double update_period = 0.5;
    int expansion_directions = 8;
    int max_frontiers_per_update = 4;
    int max_new_nodes_per_update = 8;
    int max_global_nodes = 3000;
  };

  RiskAwareGraphManager(const std::shared_ptr<EDTEnvironment>& environment,
                        const std::shared_ptr<RiskAwareEdge>& evaluator,
                        std::list<std::shared_ptr<GraphNode>>& graph);
  void init(ros::NodeHandle& nh);
  bool enabled() const { return params_.enabled; }
  double switchRatio() const { return params_.switch_ratio; }

  void seedFromPaths(const std::vector<std::vector<Eigen::Vector3d>>& paths);
  std::vector<Eigen::Vector3d> findManagedPath(
      const Eigen::Vector3d& start, const Eigen::Vector3d& goal) const;
  void setCurrentPath(const std::vector<Eigen::Vector3d>& path);
  bool updateGraphManagement(const Eigen::Vector3d& robot_pose);

 private:
  using NodePtr = std::shared_ptr<GraphNode>;
  using BucketKey = std::uint64_t;
  static BucketKey bucketKey(int x, int y);
  BucketKey bucketOf(const Eigen::Vector3d& point) const;
  std::vector<NodePtr> nearby(const Eigen::Vector3d& point, double radius) const;
  NodePtr nearest(const Eigen::Vector3d& point, double radius) const;
  NodePtr addNode(const Eigen::Vector3d& point);
  bool nodeValid(const Eigen::Vector3d& point) const;
  bool edgeValid(const Eigen::Vector3d& from, const Eigen::Vector3d& to,
                 double& risk) const;
  bool connect(const NodePtr& from, const NodePtr& to);
  std::vector<NodePtr> extractLocalNodes(const Eigen::Vector3d& pose) const;
  size_t updateNodeValidity(const std::vector<NodePtr>& nodes, double stamp);
  size_t updateLocalEdgeRisk(const std::vector<NodePtr>& nodes, double stamp);
  std::vector<NodePtr> detectFrontiers(const std::vector<NodePtr>& nodes,
                                       const Eigen::Vector3d& pose);
  size_t expandFrontiers(const std::vector<NodePtr>& frontiers,
                         const Eigen::Vector3d& pose);
  bool checkCurrentPathValid(const Eigen::Vector3d& pose) const;

  Parameters params_;
  std::shared_ptr<EDTEnvironment> environment_;
  std::shared_ptr<RiskAwareEdge> evaluator_;
  std::list<NodePtr>& graph_;
  std::unordered_map<BucketKey, std::vector<NodePtr>> buckets_;
  std::vector<Eigen::Vector3d> current_path_;
  std::vector<double> current_path_baseline_risk_;
  ros::WallTime last_update_wall_;
  int next_node_id_ = 2;
  size_t edge_count_ = 0;
};

}  // namespace fast_planner
