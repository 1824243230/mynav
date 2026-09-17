#ifndef PATH_SEARCHING_RISK_AWARE_PATH_SELECTOR_H_
#define PATH_SEARCHING_RISK_AWARE_PATH_SELECTOR_H_

#include <Eigen/Core>
#include <plan_env/risk_map_manager.h>
#include <ros/ros.h>

#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

namespace fast_planner {

struct PathSelectionCandidate {
  std::vector<Eigen::Vector3d> path;
  double length = 0.0;
  double risk = 0.0;
  double prs_score = 0.0;
};

struct PathSelectionScore {
  double route_cost = std::numeric_limits<double>::infinity();
  double utility = -std::numeric_limits<double>::infinity();
  double normalized_length = 0.0;
  double normalized_risk = 0.0;
  double normalized_prs = 0.0;
  double prs_score = 0.0;
  double orientation_error = std::numeric_limits<double>::infinity();
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
  bool fixed_scale_score = false;
  double route_cost = std::numeric_limits<double>::infinity();
  std::vector<PathSelectionScore> candidate_scores;
};

struct TopologySwitchDecision {
  bool propose_switch = false;
  bool active_topology_invalid = false;
  double gain = -std::numeric_limits<double>::infinity();
  double connection_penalty = 0.0;
  double margin = -std::numeric_limits<double>::infinity();
};

/**
 * @brief Selects the final topological guide path using risk-aware utility.
 *
 * normalized_length is a desirability value: the shortest candidate is 1 and
 * the longest is 0. normalized_risk is a penalty: the riskiest candidate is 1.
 * In legacy mode, Cost = w1 * normalized_length - w2 * normalized_risk is
 * maximized. With PRS enabled, a positive PRS term rewards reliable paths.
 * In ECTS fixed-scale mode, J_route is formed from value/reference terms and
 * minimized, so its value is independent of the other candidates present.
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
    bool ects_enabled = false;
    bool use_fixed_scale_score = true;
    double length_ref = 1.0;
    double risk_ref = 1.0;
    double prs_ref = 1.0;
    double lambda_switch = 1.0;
    double switch_margin = 0.0;
    double dubins_ref = 1.0;
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

  TopologySwitchDecision evaluateTopologySwitch(
      double keep_cost, double challenger_cost,
      double keep_dubins_length, double challenger_dubins_length,
      bool active_topology_invalid) const;

  const Parameters& getParameters() const { return params_; }

 private:
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
};

}  // namespace fast_planner

#endif  // PATH_SEARCHING_RISK_AWARE_PATH_SELECTOR_H_
