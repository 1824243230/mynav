#include <path_searching/risk_aware_path_selector.h>

#include <algorithm>
#include <cmath>

namespace fast_planner {
namespace {

constexpr double kEpsilon = 1e-9;
constexpr double kPi = 3.14159265358979323846;

double dynamicScale(double value, double threshold, double gain) {
  if (threshold <= kEpsilon || value <= threshold) {
    return 1.0;
  }
  const double excess_ratio = std::min(1.0, (value - threshold) / threshold);
  return 1.0 + std::max(0.0, gain) * excess_ratio;
}

}  // namespace

RiskAwarePathSelector::RiskAwarePathSelector(const Parameters& parameters)
    : params_(sanitizeParameters(parameters, 0.1)) {}

void RiskAwarePathSelector::init(ros::NodeHandle& nh,
                                 const RiskMapManager::Ptr& risk_map_manager,
                                 double map_resolution) {
  risk_map_manager_ = risk_map_manager;
  map_resolution_ = std::max(map_resolution, kEpsilon);

  nh.param("risk_aware_path_selector/w1", params_.w1, params_.w1);
  nh.param("risk_aware_path_selector/w2", params_.w2, params_.w2);
  nh.param("risk_aware_path_selector/high_risk_threshold",
           params_.high_risk_threshold, params_.high_risk_threshold);
  nh.param("risk_aware_path_selector/open_space_width_threshold",
           params_.open_space_width_threshold, params_.open_space_width_threshold);
  nh.param("risk_aware_path_selector/dynamic_weight_gain",
           params_.dynamic_weight_gain, params_.dynamic_weight_gain);
  nh.param("risk_aware_path_selector/orientation_tie_threshold",
           params_.orientation_tie_threshold, params_.orientation_tie_threshold);
  nh.param("risk_aware_path_selector/sample_resolution",
           params_.sample_resolution, params_.sample_resolution);
  nh.param("risk_aware_path_selector/enable_tcbs", params_.enable_tcbs,
           params_.enable_tcbs);
  nh.param("risk_aware_path_selector/eta_switch", params_.eta_switch,
           params_.eta_switch);
  nh.param("risk_aware_path_selector/switch_threshold", params_.switch_threshold,
           params_.switch_threshold);
  nh.param("risk_aware_path_selector/lambda_switch", params_.lambda_switch,
           params_.lambda_switch);
  nh.param("path_reliability/enable", params_.reliability_enabled,
           params_.reliability_enabled);
  nh.param("path_reliability/lambda_length", params_.lambda_length,
           params_.lambda_length);
  nh.param("path_reliability/lambda_risk", params_.lambda_risk,
           params_.lambda_risk);
  nh.param("path_reliability/lambda_prs", params_.lambda_prs,
           params_.lambda_prs);

  params_ = sanitizeParameters(params_, map_resolution_);

  ROS_INFO_STREAM("RiskAwarePathSelector initialized: w1=" << params_.w1
                  << ", w2=" << params_.w2
                  << ", high_risk_threshold=" << params_.high_risk_threshold
                  << ", open_space_width_threshold="
                  << params_.open_space_width_threshold
                  << ", enable_tcbs=" << std::boolalpha << params_.enable_tcbs
                  << ", eta_switch=" << params_.eta_switch
                  << ", switch_threshold=" << params_.switch_threshold
                  << ", lambda_switch=" << params_.lambda_switch
                  << ", reliability_enabled=" << std::boolalpha
                  << params_.reliability_enabled
                  << ", lambda_length=" << params_.lambda_length
                  << ", lambda_risk=" << params_.lambda_risk
                  << ", lambda_prs=" << params_.lambda_prs);
}

PathSelectionResult RiskAwarePathSelector::selectBestPath(
    const std::vector<PathSelectionCandidate>& candidates,
    double start_yaw) const {
  PathSelectionResult result;
  if (candidates.empty()) {
    return result;
  }

  double min_length = std::numeric_limits<double>::infinity();
  double max_length = -std::numeric_limits<double>::infinity();
  double min_risk = std::numeric_limits<double>::infinity();
  double max_risk = -std::numeric_limits<double>::infinity();
  double risk_sum = 0.0;
  std::size_t valid_risk_count = 0;

  for (const auto& candidate : candidates) {
    if (!std::isfinite(candidate.risk) || !std::isfinite(candidate.length)) {
      continue;
    }
    min_length = std::min(min_length, candidate.length);
    max_length = std::max(max_length, candidate.length);
    min_risk = std::min(min_risk, candidate.risk);
    max_risk = std::max(max_risk, candidate.risk);
    risk_sum += candidate.risk;
    ++valid_risk_count;
  }

  if (!std::isfinite(min_length)) {
    min_length = max_length = 0.0;
  }
  if (!std::isfinite(min_risk)) {
    min_risk = max_risk = 0.0;
  }

  result.average_risk = valid_risk_count > 0
                            ? risk_sum / static_cast<double>(valid_risk_count)
                            : 0.0;
  result.average_corridor_width = computeAverageCorridorWidth(candidates);
  result.w1 = params_.w1 * dynamicScale(result.average_corridor_width,
                                        params_.open_space_width_threshold,
                                        params_.dynamic_weight_gain);
  result.w2 = params_.w2 * dynamicScale(result.average_risk,
                                        params_.high_risk_threshold,
                                        params_.dynamic_weight_gain);
  result.reliability_enabled = params_.reliability_enabled;
  result.lambda_length = params_.lambda_length;
  result.lambda_risk = params_.lambda_risk;
  result.lambda_prs = params_.lambda_prs;

  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const auto& candidate = candidates[i];
    if (!std::isfinite(candidate.risk) || !std::isfinite(candidate.length)) {
      continue;
    }
    const double length_score =
        1.0 - normalizedValue(candidate.length, min_length, max_length, 0.0);
    const double risk_penalty =
        normalizedValue(candidate.risk, min_risk, max_risk, 0.0);
    const double prs_score = params_.reliability_enabled &&
                                     std::isfinite(candidate.prs_score)
                                 ? std::max(0.0, std::min(1.0, candidate.prs_score))
                                 : 0.0;
    const double cost = params_.reliability_enabled
                            ? params_.lambda_length * length_score -
                                  params_.lambda_risk * risk_penalty +
                                  params_.lambda_prs * prs_score
                            : result.w1 * length_score - result.w2 * risk_penalty;
    const double orientation_error = initialHeadingError(candidate.path, start_yaw);

    const bool higher_cost = !result.success ||
                             cost > result.cost + params_.orientation_tie_threshold;
    const bool orientation_preferred = result.success &&
        std::abs(cost - result.cost) <= params_.orientation_tie_threshold &&
        orientation_error < result.orientation_error;
    if (higher_cost || orientation_preferred) {
      result.success = true;
      result.best_index = i;
      result.best_path = candidate.path;
      result.cost = cost;
      result.normalized_length = length_score;
      result.normalized_risk = risk_penalty;
      result.prs_score = prs_score;
      result.orientation_error = orientation_error;
    }
  }

  return result;
}

PathSelectionResult RiskAwarePathSelector::selectStablePath(
    const std::vector<PathSelectionCandidate>& candidates) const {
  PathSelectionResult result;
  result.current_topology_id = current_topology_id_;
  std::size_t current_index = candidates.size();
  std::size_t proposed_index = candidates.size();
  double current_cost = std::numeric_limits<double>::infinity();
  double proposed_cost = std::numeric_limits<double>::infinity();

  for (std::size_t index = 0; index < candidates.size(); ++index) {
    const auto& candidate = candidates[index];
    if (candidate.topology_id == 0 || !std::isfinite(candidate.path_cost) ||
        !std::isfinite(candidate.risk) || candidate.path_cost < 0.0) {
      continue;
    }
    const bool same_topology = candidate.topology_id == current_topology_id_;
    if (same_topology && candidate.path_cost < current_cost) {
      current_cost = candidate.path_cost;
      current_index = index;
    }
    const double switch_cost = current_topology_id_ != 0 && !same_topology
                                   ? params_.lambda_switch : 0.0;
    const double total_cost = candidate.path_cost + switch_cost;
    if (total_cost < proposed_cost ||
        (total_cost == proposed_cost && same_topology)) {
      proposed_cost = total_cost;
      proposed_index = index;
    }
  }

  if (proposed_index == candidates.size()) {
    result.switch_decision = "NO_VALID_CANDIDATE";
    ROS_INFO_STREAM("[TopologyStability] current=" << current_topology_id_
                    << " candidate=0 decision=" << result.switch_decision
                    << " gain=0");
    return result;
  }

  result.candidate_topology_id = candidates[proposed_index].topology_id;
  std::size_t selected_index = proposed_index;
  if (current_topology_id_ == 0) {
    result.switch_decision = "INITIAL";
  } else if (proposed_index == current_index) {
    result.switch_decision = "KEEP";
  } else if (current_index == candidates.size()) {
    result.switch_decision = "FORCE_SWITCH";
  } else {
    result.gain = (current_cost - proposed_cost) / (current_cost + kEpsilon);
    if (result.gain > params_.switch_threshold) {
      result.switch_decision = "SWITCH";
    } else {
      selected_index = current_index;
      result.switch_decision = "KEEP";
    }
  }

  result.success = true;
  result.best_index = selected_index;
  result.best_path = candidates[selected_index].path;
  result.switch_cost = current_topology_id_ != 0 &&
                               candidates[selected_index].topology_id != current_topology_id_
                           ? params_.lambda_switch : 0.0;
  result.cost = candidates[selected_index].path_cost + result.switch_cost;
  ROS_INFO_STREAM("[TopologyStability] current=" << current_topology_id_
                  << " candidate=" << result.candidate_topology_id
                  << " selected=" << candidates[selected_index].topology_id
                  << " decision=" << result.switch_decision
                  << " gain=" << result.gain
                  << " J_current=" << current_cost
                  << " J_candidate=" << proposed_cost
                  << " J_total=" << result.cost);
  return result;
}

void RiskAwarePathSelector::commitTopology(std::uint64_t topology_id) {
  if (topology_id != 0) current_topology_id_ = topology_id;
}

TCBSScore RiskAwarePathSelector::evaluateTCBSScore(
    const std::vector<Eigen::Vector3d>& path,
    double length,
    double risk) const {
  TCBSScore score;
  score.efficiency = computeEfficiency(path, length);

  // Reuse the existing path risk and normalize it with the fixed physical
  // high-risk threshold. Candidate-set min/max normalization is intentionally
  // not used because its scale changes between replanning cycles.
  if (!std::isfinite(score.efficiency) || !std::isfinite(risk) || risk < 0.0 ||
      !std::isfinite(params_.high_risk_threshold) ||
      params_.high_risk_threshold <= kEpsilon) {
    return score;
  }
  score.risk = std::max(0.0, std::min(1.0, risk / params_.high_risk_threshold));

  // TCBS bottleneck score: the worse of efficiency and normalized risk.
  score.bottleneck = std::max(score.efficiency, score.risk);
  score.feasible = std::isfinite(score.bottleneck);
  return score;
}

double RiskAwarePathSelector::computeEfficiency(
    const std::vector<Eigen::Vector3d>& path,
    double length) {
  if (path.size() < 2 || !std::isfinite(length) || length <= kEpsilon) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const double direct_distance =
      (path.back() - path.front()).head<2>().norm();
  if (!std::isfinite(direct_distance)) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  // Efficiency penalty: zero is direct, one is maximally inefficient.
  const double efficiency = 1.0 - direct_distance / (length + kEpsilon);
  if (!std::isfinite(efficiency)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::max(0.0, std::min(1.0, efficiency));
}

double RiskAwarePathSelector::computeAverageCorridorWidth(
    const std::vector<PathSelectionCandidate>& candidates) const {
  if (!risk_map_manager_ || !risk_map_manager_->isReady()) {
    return 0.0;
  }

  double width_sum = 0.0;
  std::size_t sample_count = 0;
  for (const auto& candidate : candidates) {
    if (!std::isfinite(candidate.risk) || !std::isfinite(candidate.length)) {
      continue;
    }
    for (std::size_t i = 0; i + 1 < candidate.path.size(); ++i) {
      const Eigen::Vector2d start = candidate.path[i].head<2>();
      const Eigen::Vector2d end = candidate.path[i + 1].head<2>();
      const double length = (end - start).norm();
      const int steps = std::max(1, static_cast<int>(std::ceil(
          length / params_.sample_resolution)));
      for (int step = 0; step <= steps; ++step) {
        const double ratio = static_cast<double>(step) / static_cast<double>(steps);
        const double width = risk_map_manager_->getCorridorWidth(
            start + ratio * (end - start));
        if (std::isfinite(width) && width > 0.0) {
          width_sum += width;
          ++sample_count;
        }
      }
    }
  }
  return sample_count > 0 ? width_sum / static_cast<double>(sample_count) : 0.0;
}

double RiskAwarePathSelector::initialHeadingError(
    const std::vector<Eigen::Vector3d>& path, double start_yaw) {
  if (path.size() < 2) {
    return std::numeric_limits<double>::infinity();
  }
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector2d direction = path[i].head<2>() - path[0].head<2>();
    if (direction.squaredNorm() > kEpsilon) {
      return std::abs(wrapAngle(std::atan2(direction.y(), direction.x()) - start_yaw));
    }
  }
  return std::numeric_limits<double>::infinity();
}

double RiskAwarePathSelector::wrapAngle(double angle) {
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

double RiskAwarePathSelector::normalizedValue(double value,
                                              double minimum,
                                              double maximum,
                                              double equal_value) {
  const double range = maximum - minimum;
  if (range <= kEpsilon) {
    return equal_value;
  }
  return std::max(0.0, std::min(1.0, (value - minimum) / range));
}

RiskAwarePathSelector::Parameters RiskAwarePathSelector::sanitizeParameters(
    const Parameters& parameters, double map_resolution) {
  Parameters sanitized = parameters;
  if (!std::isfinite(sanitized.w1) || sanitized.w1 < 0.0) sanitized.w1 = 1.0;
  if (!std::isfinite(sanitized.w2) || sanitized.w2 < 0.0) sanitized.w2 = 1.0;
  if (!std::isfinite(sanitized.lambda_length) || sanitized.lambda_length < 0.0) {
    sanitized.lambda_length = 1.0;
  }
  if (!std::isfinite(sanitized.lambda_risk) || sanitized.lambda_risk < 0.0) {
    sanitized.lambda_risk = 1.0;
  }
  if (!std::isfinite(sanitized.lambda_prs) || sanitized.lambda_prs < 0.0) {
    sanitized.lambda_prs = 1.0;
  }
  if (!std::isfinite(sanitized.orientation_tie_threshold) ||
      sanitized.orientation_tie_threshold < 0.0) {
    sanitized.orientation_tie_threshold = 0.05;
  }
  if (!std::isfinite(sanitized.eta_switch)) {
    sanitized.eta_switch = 0.15;
  }
  sanitized.eta_switch = std::max(0.0, std::min(1.0, sanitized.eta_switch));
  if (!std::isfinite(sanitized.switch_threshold) ||
      sanitized.switch_threshold < 0.0) {
    sanitized.switch_threshold = Parameters().switch_threshold;
  }
  if (!std::isfinite(sanitized.lambda_switch) ||
      sanitized.lambda_switch < 0.0) {
    sanitized.lambda_switch = Parameters().lambda_switch;
  }
  if (!std::isfinite(sanitized.high_risk_threshold) ||
      sanitized.high_risk_threshold <= kEpsilon) {
    sanitized.high_risk_threshold = Parameters().high_risk_threshold;
  }
  const double valid_map_resolution =
      std::isfinite(map_resolution) ? std::max(map_resolution, kEpsilon) : 0.1;
  if (!std::isfinite(sanitized.sample_resolution) ||
      sanitized.sample_resolution <= 0.0) {
    sanitized.sample_resolution = valid_map_resolution;
  } else {
    sanitized.sample_resolution =
        std::max(sanitized.sample_resolution, valid_map_resolution);
  }
  return sanitized;
}

}  // namespace fast_planner
