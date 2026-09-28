#ifndef PATH_SEARCHING_RISK_AWARE_PATH_SELECTOR_H_
#define PATH_SEARCHING_RISK_AWARE_PATH_SELECTOR_H_

#include <Eigen/Core>
#include <plan_env/risk_map_manager.h>
#include <ros/ros.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

namespace fast_planner {

struct PathSelectionCandidate {
  std::vector<Eigen::Vector3d> path;
  double length = 0.0;
  double risk = 0.0;
  double prs_score = 0.0;
  std::uint64_t topology_id = 0;
  double path_cost = std::numeric_limits<double>::infinity();
};

// Lightweight TCBS evaluation data. It is intentionally not used by the
// legacy selector until TCBS is explicitly enabled and implemented.
struct TCBSScore {
  double efficiency = 0.0;
  double risk = 0.0;
  double bottleneck = std::numeric_limits<double>::infinity();
  bool feasible = false;
};

struct PathSelectionResult {
  bool success = false;
  std::size_t best_index = 0;
  std::vector<Eigen::Vector3d> best_path;
  double cost = -std::numeric_limits<double>::infinity();
  double normalized_length = 0.0;
  double normalized_risk = 0.0;
  double prs_score = 0.0;
  double orientation_error = std::numeric_limits<double>::infinity();
  double w1 = 1.0;
  double w2 = 1.0;
  double average_risk = 0.0;
  double average_corridor_width = 0.0;
  bool reliability_enabled = false;
  double lambda_length = 1.0;
  double lambda_risk = 1.0;
  double lambda_prs = 1.0;
  std::uint64_t current_topology_id = 0;
  std::uint64_t candidate_topology_id = 0;
  double switch_cost = 0.0;
  double gain = 0.0;
  const char* switch_decision = "NONE";
};

/**
 * @brief Selects the final topological guide path using risk-aware utility.
 *
 * normalized_length is a desirability value: the shortest candidate is 1 and
 * the longest is 0. normalized_risk is a penalty: the riskiest candidate is 1.
 * In legacy mode, Cost = w1 * normalized_length - w2 * normalized_risk is
 * maximized. With PRS enabled, a positive PRS term rewards reliable paths.
 * Initial-heading error remains an orientation-aware tie breaker.
 */
class RiskAwarePathSelector {
 public:
  using Ptr = std::shared_ptr<RiskAwarePathSelector>;

  struct Parameters {
    double w1 = 1.0;
    double w2 = 1.0;
    double high_risk_threshold = 5.0;
    double open_space_width_threshold = 2.0;
    double dynamic_weight_gain = 1.0;
    double orientation_tie_threshold = 0.05;
    double sample_resolution = 0.1;
    bool reliability_enabled = false;
    double lambda_length = 1.0;
    double lambda_risk = 1.0;
    double lambda_prs = 1.0;
    bool enable_tcbs = false;
    double eta_switch = 0.15;
    double switch_threshold = 0.1;
    double lambda_switch = 0.1;
  };

  RiskAwarePathSelector() = default;
  explicit RiskAwarePathSelector(const Parameters& parameters);
  ~RiskAwarePathSelector() = default;

  void init(ros::NodeHandle& nh,
            const RiskMapManager::Ptr& risk_map_manager,
            double map_resolution);

  PathSelectionResult selectBestPath(
      const std::vector<PathSelectionCandidate>& candidates,
      double start_yaw) const;

  // Uses supplied J_path and topology IDs to apply switch cost and gain gate.
  // A proposal does not change the current topology; commit only after the
  // downstream trajectory is accepted.
  PathSelectionResult selectStablePath(
      const std::vector<PathSelectionCandidate>& candidates) const;
  void commitTopology(std::uint64_t topology_id);
  void resetTopology() { current_topology_id_ = 0; }
  std::uint64_t currentTopologyId() const { return current_topology_id_; }

  TCBSScore evaluateTCBSScore(const std::vector<Eigen::Vector3d>& path,
                              double length,
                              double risk) const;

  const Parameters& getParameters() const { return params_; }
  // 可选图管理启用时复用现有 TCBS Keep/Challenger 滞回。
  void enableTCBSHysteresis(double eta) {
    params_.enable_tcbs = true;
    params_.eta_switch = std::max(0.0, std::min(1.0, eta));
  }

 private:
  static double computeEfficiency(const std::vector<Eigen::Vector3d>& path,
                                  double length);
  double computeAverageCorridorWidth(
      const std::vector<PathSelectionCandidate>& candidates) const;
  static double initialHeadingError(const std::vector<Eigen::Vector3d>& path,
                                    double start_yaw);
  static double wrapAngle(double angle);
  static double normalizedValue(double value, double minimum, double maximum,
                                double equal_value);
  static Parameters sanitizeParameters(const Parameters& parameters,
                                       double map_resolution);

  Parameters params_;
  RiskMapManager::Ptr risk_map_manager_;
  double map_resolution_ = 0.1;
  std::uint64_t current_topology_id_ = 0;
};

}  // namespace fast_planner

#endif  // PATH_SEARCHING_RISK_AWARE_PATH_SELECTOR_H_
