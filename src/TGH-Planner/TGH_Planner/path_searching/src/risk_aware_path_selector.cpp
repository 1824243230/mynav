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
  nh.param("path_reliability/enable", params_.reliability_enabled,
           params_.reliability_enabled);
  nh.param("path_reliability/lambda_length", params_.lambda_length,
           params_.lambda_length);
  nh.param("path_reliability/lambda_risk", params_.lambda_risk,
           params_.lambda_risk);
  nh.param("path_reliability/lambda_prs", params_.lambda_prs,
           params_.lambda_prs);
  nh.param("ects/enabled", params_.ects_enabled, params_.ects_enabled);
  nh.param("ects/use_fixed_scale_score", params_.use_fixed_scale_score,
           params_.use_fixed_scale_score);
  nh.param("ects/length_ref", params_.length_ref, params_.length_ref);
  nh.param("ects/risk_ref", params_.risk_ref, params_.risk_ref);
  nh.param("ects/prs_ref", params_.prs_ref, params_.prs_ref);
  nh.param("ects/lambda_switch", params_.lambda_switch, params_.lambda_switch);
  nh.param("ects/switch_margin", params_.switch_margin, params_.switch_margin);
  nh.param("ects/dubins_ref", params_.dubins_ref, params_.dubins_ref);

  params_ = sanitizeParameters(params_, map_resolution_);

  ROS_INFO_STREAM("RiskAwarePathSelector initialized: w1=" << params_.w1
                  << ", w2=" << params_.w2
                  << ", high_risk_threshold=" << params_.high_risk_threshold
                  << ", open_space_width_threshold="
                  << params_.open_space_width_threshold
                  << ", reliability_enabled=" << std::boolalpha
                  << params_.reliability_enabled
                  << ", lambda_length=" << params_.lambda_length
                  << ", lambda_risk=" << params_.lambda_risk
                  << ", lambda_prs=" << params_.lambda_prs
                  << ", ects_enabled=" << params_.ects_enabled
                  << ", fixed_scale_score=" << params_.use_fixed_scale_score
                  << ", length_ref=" << params_.length_ref
                  << ", risk_ref=" << params_.risk_ref
                  << ", prs_ref=" << params_.prs_ref
                  << ", lambda_switch=" << params_.lambda_switch
                  << ", switch_margin=" << params_.switch_margin
                  << ", dubins_ref=" << params_.dubins_ref);
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
    if (std::isfinite(candidate.length)) {
      min_length = std::min(min_length, candidate.length);
      max_length = std::max(max_length, candidate.length);
    }
    if (std::isfinite(candidate.risk)) {
      min_risk = std::min(min_risk, candidate.risk);
      max_risk = std::max(max_risk, candidate.risk);
      risk_sum += candidate.risk;
      ++valid_risk_count;
    }
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
  result.fixed_scale_score = params_.ects_enabled && params_.use_fixed_scale_score;
  result.candidate_scores.reserve(candidates.size());

  for (std::size_t i = 0; i < candidates.size(); ++i) {
    const auto& candidate = candidates[i];
    const double prs_score = params_.reliability_enabled &&
                                     std::isfinite(candidate.prs_score)
                                 ? std::max(0.0, std::min(1.0, candidate.prs_score))
                                 : 0.0;
    PathSelectionScore score;
    score.prs_score = prs_score;
    score.orientation_error = initialHeadingError(candidate.path, start_yaw);

    if (result.fixed_scale_score) {
      score.normalized_length = std::isfinite(candidate.length)
                                    ? candidate.length / params_.length_ref
                                    : std::numeric_limits<double>::infinity();
      score.normalized_risk = std::isfinite(candidate.risk)
                                  ? candidate.risk / params_.risk_ref
                                  : std::numeric_limits<double>::infinity();
      score.normalized_prs = params_.reliability_enabled
                                 ? prs_score / params_.prs_ref
                                 : 0.0;
      score.route_cost = params_.reliability_enabled
                             ? params_.lambda_length * score.normalized_length +
                                   params_.lambda_risk * score.normalized_risk +
                                   params_.lambda_prs * score.normalized_prs
                             : params_.w1 * score.normalized_length +
                                   params_.w2 * score.normalized_risk;
      score.utility = -score.route_cost;
    } else {
      score.normalized_length = std::isfinite(candidate.length)
                                    ? 1.0 - normalizedValue(candidate.length, min_length,
                                                            max_length, 0.0)
                                    : 0.0;
      score.normalized_risk = std::isfinite(candidate.risk)
                                  ? normalizedValue(candidate.risk, min_risk,
                                                    max_risk, 0.0)
                                  : 1.0;
      score.normalized_prs = prs_score;
      score.utility = params_.reliability_enabled
                          ? params_.lambda_length * score.normalized_length -
                                params_.lambda_risk * score.normalized_risk +
                                params_.lambda_prs * prs_score
                          : result.w1 * score.normalized_length -
                                result.w2 * score.normalized_risk;
      score.route_cost = -score.utility;
    }
    result.candidate_scores.push_back(score);

    const bool higher_cost = !result.success ||
                             score.utility > result.cost + params_.orientation_tie_threshold;
    const bool orientation_preferred = result.success &&
        std::abs(score.utility - result.cost) <= params_.orientation_tie_threshold &&
        score.orientation_error < result.orientation_error;
    if (higher_cost || orientation_preferred) {
      result.success = true;
      result.best_index = i;
      result.best_path = candidate.path;
      result.cost = score.utility;
      result.route_cost = score.route_cost;
      result.normalized_length = score.normalized_length;
      result.normalized_risk = score.normalized_risk;
      result.prs_score = prs_score;
      result.orientation_error = score.orientation_error;
    }
  }

  return result;
}

TopologySwitchDecision RiskAwarePathSelector::evaluateTopologySwitch(
    double keep_cost, double challenger_cost,
    double keep_dubins_length, double challenger_dubins_length,
    bool active_topology_invalid) const {
  TopologySwitchDecision decision;
  decision.active_topology_invalid = active_topology_invalid;
  decision.gain = (keep_cost - challenger_cost) / (keep_cost + 1e-6);

  if (!active_topology_invalid) {
    if (std::isfinite(keep_dubins_length) &&
        std::isfinite(challenger_dubins_length)) {
      decision.connection_penalty =
          std::max(0.0, challenger_dubins_length - keep_dubins_length) /
          params_.dubins_ref;
    } else if (!std::isfinite(challenger_dubins_length)) {
      decision.connection_penalty = std::numeric_limits<double>::infinity();
    }
  }

  decision.margin = decision.gain -
                    params_.lambda_switch * decision.connection_penalty;
  decision.propose_switch = decision.gain > 0.0 &&
                            decision.margin > params_.switch_margin;
  return decision;
}

double RiskAwarePathSelector::computeAverageCorridorWidth(
    const std::vector<PathSelectionCandidate>& candidates) const {
  if (!risk_map_manager_ || !risk_map_manager_->isReady()) {
    return 0.0;
  }

  double width_sum = 0.0;
  std::size_t sample_count = 0;
  for (const auto& candidate : candidates) {
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
  if (!std::isfinite(sanitized.length_ref) || sanitized.length_ref <= kEpsilon) {
    sanitized.length_ref = 1.0;
  }
  if (!std::isfinite(sanitized.risk_ref) || sanitized.risk_ref <= kEpsilon) {
    sanitized.risk_ref = 1.0;
  }
  if (!std::isfinite(sanitized.prs_ref) || sanitized.prs_ref <= kEpsilon) {
    sanitized.prs_ref = 1.0;
  }
  if (!std::isfinite(sanitized.lambda_switch) || sanitized.lambda_switch < 0.0) {
    sanitized.lambda_switch = 1.0;
  }
  if (!std::isfinite(sanitized.switch_margin)) {
    sanitized.switch_margin = 0.0;
  }
  if (!std::isfinite(sanitized.dubins_ref) || sanitized.dubins_ref <= kEpsilon) {
    sanitized.dubins_ref = 1.0;
  }
  if (!std::isfinite(sanitized.orientation_tie_threshold) ||
      sanitized.orientation_tie_threshold < 0.0) {
    sanitized.orientation_tie_threshold = 0.05;
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
