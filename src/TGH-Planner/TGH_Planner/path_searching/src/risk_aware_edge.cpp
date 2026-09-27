#include <path_searching/risk_aware_edge.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <utility>

namespace fast_planner {

void RiskAwareEdge::init(ros::NodeHandle& nh,
                         const RiskMapManager::Ptr& risk_map_manager,
                         double map_resolution) {
  nh.param("risk_aware_graph/alpha", params_.alpha, 1.0);
  nh.param("risk_aware_graph/beta", params_.beta, 1.0);
  nh.param("risk_aware_graph/gamma", params_.gamma, 0.0);
  nh.param("risk_aware_graph/edge_sample_resolution", params_.sample_resolution,
           map_resolution);
  nh.param("risk_aware_graph/fine_resolution", params_.sample_resolution,
           params_.sample_resolution);
  nh.param("risk_aware_graph/coarse_resolution", params_.coarse_resolution,
           params_.coarse_resolution);
  nh.param("risk_aware_graph/top_k_path", params_.top_k_path,
           params_.top_k_path);
  nh.param("risk_aware_graph/normalization_epsilon", params_.normalization_epsilon,
           params_.normalization_epsilon);
  nh.param("risk_aware_graph/risk_cvar_ratio", params_.risk_cvar_ratio,
           params_.risk_cvar_ratio);
  nh.param("risk_aware_graph/risk_cvar_weight", params_.risk_cvar_weight,
           params_.risk_cvar_weight);
  nh.param("risk_aware_graph/risk_threshold", params_.risk_threshold,
           params_.risk_threshold);
  // Reuse the risk-map scale already configured for the selector when the new
  // graph threshold is absent from an older configuration.
  nh.param("risk_aware_path_selector/high_risk_threshold",
           params_.risk_safe_threshold, params_.risk_safe_threshold);
  nh.param("risk_aware_graph/risk_safe_threshold",
           params_.risk_safe_threshold, params_.risk_safe_threshold);
  nh.param("risk_aware_graph/nominal_speed", params_.nominal_speed,
           params_.nominal_speed);
  nh.param("risk_aware_graph/min_turn_radius", params_.min_turn_radius,
           params_.min_turn_radius);

  params_.alpha = std::max(0.0, params_.alpha);
  params_.beta = std::max(0.0, params_.beta);
  params_.gamma = std::max(0.0, params_.gamma);
  if (!std::isfinite(params_.sample_resolution) || params_.sample_resolution <= 0.0) {
    params_.sample_resolution = std::max(1e-3, map_resolution);
  }
  if (!std::isfinite(params_.coarse_resolution) ||
      params_.coarse_resolution <= 0.0) {
    params_.coarse_resolution = Parameters().coarse_resolution;
  }
  params_.coarse_resolution = std::max(params_.coarse_resolution,
                                       params_.sample_resolution);
  if (params_.top_k_path <= 0) params_.top_k_path = Parameters().top_k_path;
  if (!std::isfinite(params_.normalization_epsilon) ||
      params_.normalization_epsilon <= 0.0) {
    params_.normalization_epsilon = Parameters().normalization_epsilon;
  }
  if (!std::isfinite(params_.risk_cvar_ratio) ||
      params_.risk_cvar_ratio <= 0.0 || params_.risk_cvar_ratio > 1.0) {
    params_.risk_cvar_ratio = Parameters().risk_cvar_ratio;
  }
  if (!std::isfinite(params_.risk_cvar_weight) || params_.risk_cvar_weight < 0.0) {
    params_.risk_cvar_weight = Parameters().risk_cvar_weight;
  }
  if (!std::isfinite(params_.risk_threshold) || params_.risk_threshold < 0.0) {
    params_.risk_threshold = Parameters().risk_threshold;
  }
  if (!std::isfinite(params_.risk_safe_threshold) ||
      params_.risk_safe_threshold <= 0.0) {
    params_.risk_safe_threshold = Parameters().risk_safe_threshold;
  }
  if (!std::isfinite(params_.nominal_speed) || params_.nominal_speed <= 0.0) {
    params_.nominal_speed = Parameters().nominal_speed;
  }
  if (!std::isfinite(params_.min_turn_radius) || params_.min_turn_radius <= 0.0) {
    params_.min_turn_radius = Parameters().min_turn_radius;
  }
  risk_map_manager_ = risk_map_manager;

  ROS_INFO_STREAM("RiskAwareEdge initialized: alpha=" << params_.alpha
                  << ", beta=" << params_.beta
                  << ", gamma=" << params_.gamma
                  << ", sample_resolution=" << params_.sample_resolution
                  << ", normalization_epsilon=" << params_.normalization_epsilon
                  << ", risk_cvar_ratio=" << params_.risk_cvar_ratio
                  << ", risk_cvar_weight=" << params_.risk_cvar_weight
                  << ", risk_threshold=" << params_.risk_threshold
                  << ", risk_safe_threshold=" << params_.risk_safe_threshold
                  << ", nominal_speed=" << params_.nominal_speed
                  << ", min_turn_radius=" << params_.min_turn_radius);
  ROS_INFO_STREAM("RiskAwareEdge two-stage: coarse_resolution="
                  << params_.coarse_resolution
                  << ", fine_resolution=" << params_.sample_resolution
                  << ", top_k_path=" << params_.top_k_path);
}

RiskEdge RiskAwareEdge::evaluateEdge(NodeID start,
                                     NodeID end,
                                     const Eigen::Vector3d& start_position,
                                     const Eigen::Vector3d& end_position,
                                     const Eigen::Vector3d* previous_position) const {
  RiskEdge edge;
  edge.start = start;
  edge.end = end;
  edge.length = (end_position - start_position).head<2>().norm();
  edge.risk_cost = sampleEdgeRisk(start_position, end_position, edge.length,
                                  &edge.maximum_risk);
  if (previous_position != nullptr) {
    edge.avc_cost =
        computeAVCCost(*previous_position, start_position, end_position);
  }
  edge.curvature_cost = edge.avc_cost;
  edge.total_cost = std::isfinite(edge.risk_cost)
                        ? params_.alpha * edge.length +
                              params_.beta * edge.risk_cost +
                              params_.gamma * edge.avc_cost
                        : std::numeric_limits<double>::infinity();
  return edge;
}

RiskPathCost RiskAwareEdge::evaluatePath(const std::vector<Eigen::Vector3d>& path) const {
  RiskPathCost path_cost;
  if (path.size() < 2) {
    return path_cost;
  }

  path_cost.edges.reserve(path.size() - 1);
  double risk_sum = 0.0;
  double avc_sum = 0.0;
  for (size_t index = 0; index + 1 < path.size(); ++index) {
    const Eigen::Vector3d* previous = index > 0 ? &path[index - 1] : nullptr;
    RiskEdge edge = evaluateEdge(static_cast<NodeID>(index),
                                 static_cast<NodeID>(index + 1),
                                 path[index], path[index + 1], previous);
    path_cost.length += edge.length;
    risk_sum += edge.risk_cost;
    path_cost.maximum_risk = std::max(path_cost.maximum_risk, edge.maximum_risk);
    avc_sum += edge.avc_cost;
    path_cost.edges.emplace_back(std::move(edge));
  }

  const double edge_count = static_cast<double>(path_cost.edges.size());
  path_cost.risk = risk_sum / edge_count;
  path_cost.avc_cost = avc_sum / edge_count;
  path_cost.curvature_cost = path_cost.avc_cost;
  path_cost.total_cost = std::isfinite(path_cost.risk)
                             ? params_.alpha * path_cost.length +
                                   params_.beta * path_cost.risk +
                                   params_.gamma * path_cost.avc_cost
                             : std::numeric_limits<double>::infinity();
  return path_cost;
}

void RiskAwareEdge::evaluateCandidatePaths(
    const std::vector<std::vector<Eigen::Vector3d>>& paths,
    std::vector<std::size_t>& selected_indices,
    std::vector<RiskPathCost>& fine_costs) const {
  using Clock = std::chrono::steady_clock;
  const auto start_time = Clock::now();
  selected_indices.clear();
  fine_costs.clear();

  struct CoarseCandidate {
    double length;
    double risk;
    double score;
    std::size_t index;
  };
  std::vector<CoarseCandidate> coarse_candidates;
  coarse_candidates.reserve(paths.size());
  double length_max = 0.0;
  double risk_max = 0.0;
  for (std::size_t candidate_index = 0; candidate_index < paths.size();
       ++candidate_index) {
    const auto& path = paths[candidate_index];
    if (path.size() < 2) {
      coarse_candidates.push_back({0.0, 0.0,
                                   std::numeric_limits<double>::infinity(),
                                   candidate_index});
      continue;
    }
    double length = 0.0;
    double risk_sum = 0.0;
    for (std::size_t index = 0; index + 1 < path.size(); ++index) {
      const double edge_length = (path[index + 1] - path[index]).head<2>().norm();
      length += edge_length;
      risk_sum += sampleEdgeAverageRisk(path[index], path[index + 1],
                                        edge_length, params_.coarse_resolution);
    }
    const double edge_count = static_cast<double>(path.size() > 1 ? path.size() - 1 : 0);
    const double risk = edge_count > 0.0 ? risk_sum / edge_count : 0.0;
    coarse_candidates.push_back({length, risk, 0.0, candidate_index});
    if (std::isfinite(length) && std::isfinite(risk)) {
      length_max = std::max(length_max, length);
      risk_max = std::max(risk_max, risk);
    }
  }

  for (auto& candidate : coarse_candidates) {
    candidate.score = paths[candidate.index].size() >= 2 &&
                              std::isfinite(candidate.length) &&
                              std::isfinite(candidate.risk)
                          ? params_.alpha * candidate.length /
                                (length_max + params_.normalization_epsilon) +
                                params_.beta * candidate.risk /
                                (risk_max + params_.normalization_epsilon)
                          : std::numeric_limits<double>::infinity();
  }
  const std::size_t feasible_count = static_cast<std::size_t>(std::count_if(
      coarse_candidates.begin(), coarse_candidates.end(),
      [](const CoarseCandidate& candidate) {
        return std::isfinite(candidate.score);
      }));
  const std::size_t fine_count =
      std::min(feasible_count, static_cast<std::size_t>(params_.top_k_path));
  std::partial_sort(coarse_candidates.begin(),
                    coarse_candidates.begin() + fine_count,
                    coarse_candidates.end(),
                    [](const CoarseCandidate& lhs, const CoarseCandidate& rhs) {
                      return lhs.score < rhs.score ||
                             (lhs.score == rhs.score && lhs.index < rhs.index);
                    });
  const auto coarse_end = Clock::now();
  selected_indices.reserve(fine_count);
  fine_costs.reserve(fine_count);
  for (std::size_t index = 0; index < fine_count; ++index) {
    const std::size_t path_index = coarse_candidates[index].index;
    RiskPathCost fine_cost = evaluatePath(paths[path_index]);
    if (!std::isfinite(fine_cost.total_cost) ||
        fine_cost.maximum_risk > params_.risk_safe_threshold) {
      continue;
    }
    selected_indices.push_back(path_index);
    fine_costs.push_back(std::move(fine_cost));
  }
  normalizeCandidateCosts(fine_costs);
  const auto end_time = Clock::now();
  const double coarse_ms =
      std::chrono::duration<double, std::milli>(coarse_end - start_time).count();
  const double fine_ms =
      std::chrono::duration<double, std::milli>(end_time - coarse_end).count();
  ROS_DEBUG_STREAM("RiskAwareEdge two-stage: coarse_count=" << paths.size()
                   << " fine_count=" << fine_count
                   << " feasible_count=" << selected_indices.size()
                   << " coarse_ms=" << coarse_ms
                   << " fine_ms=" << fine_ms
                   << " total_ms=" << coarse_ms + fine_ms);
  ROS_INFO_STREAM_THROTTLE(1.0, "RiskAwareEdge two-stage: coarse_count="
                           << paths.size() << " fine_count=" << fine_count
                           << " feasible_count=" << selected_indices.size()
                           << " coarse_ms=" << coarse_ms
                           << " fine_ms=" << fine_ms
                           << " total_ms=" << coarse_ms + fine_ms);
}

double RiskAwareEdge::sampleEdgeAverageRisk(const Eigen::Vector3d& start,
                                             const Eigen::Vector3d& end,
                                             double length,
                                             double resolution) const {
  if (!risk_map_manager_ || !risk_map_manager_->isEnabled()) return 0.0;
  if (!risk_map_manager_->isReady()) return 0.0;
  const int segment_count =
      std::max(1, static_cast<int>(std::ceil(length / resolution)));
  double risk_sum = 0.0;
  for (int sample = 0; sample <= segment_count; ++sample) {
    const double ratio = static_cast<double>(sample) / segment_count;
    const Eigen::Vector2d position =
        ((1.0 - ratio) * start + ratio * end).head<2>();
    const double risk = risk_map_manager_->getRisk(position);
    if (!std::isfinite(risk) ||
        risk > std::min(params_.risk_threshold, params_.risk_safe_threshold)) {
      return std::numeric_limits<double>::infinity();
    }
    risk_sum += risk;
  }
  return risk_sum / static_cast<double>(segment_count + 1);
}

void RiskAwareEdge::normalizeCandidateCosts(std::vector<RiskPathCost>& costs) const {
  double length_max = 0.0;
  double risk_max = 0.0;
  for (const auto& cost : costs) {
    if (cost.maximum_risk > params_.risk_safe_threshold ||
        !std::isfinite(cost.risk)) {
      continue;
    }
    if (std::isfinite(cost.length)) length_max = std::max(length_max, cost.length);
    if (std::isfinite(cost.risk)) risk_max = std::max(risk_max, cost.risk);
  }

  const double epsilon = params_.normalization_epsilon;
  for (std::size_t index = 0; index < costs.size(); ++index) {
    auto& cost = costs[index];
    // TopoPath reconstructs stored costs through the legacy curvature field.
    if (cost.avc_cost == 0.0) cost.avc_cost = cost.curvature_cost;
    const double raw_cost = cost.total_cost;
    const bool valid = std::isfinite(cost.length) &&
                       std::isfinite(cost.risk) &&
                       std::isfinite(cost.avc_cost) &&
                       cost.maximum_risk <= params_.risk_safe_threshold;
    const double length_norm = valid ? cost.length / (length_max + epsilon) : 0.0;
    const double risk_norm = valid ? cost.risk / (risk_max + epsilon) : 0.0;
    cost.total_cost = valid
                          ? params_.alpha * length_norm +
                                params_.beta * risk_norm +
                                params_.gamma * cost.avc_cost
                          : std::numeric_limits<double>::infinity();
    ROS_DEBUG_STREAM("RiskAwareEdge candidate=" << index
                     << " raw_cost={length:" << cost.length
                     << ", risk:" << cost.risk
                     << ", avc:" << cost.avc_cost
                     << ", total:" << raw_cost << "}"
                     << " normalized_cost={length:" << length_norm
                     << ", risk:" << risk_norm
                     << "}"
                     << " avc_cost=" << cost.avc_cost
                     << " final_cost=" << cost.total_cost);
  }
}

double RiskAwareEdge::sampleEdgeRisk(const Eigen::Vector3d& start,
                                     const Eigen::Vector3d& end,
                                     double length,
                                     double* maximum_risk) const {
  if (maximum_risk != nullptr) *maximum_risk = 0.0;
  if (!risk_map_manager_ || !risk_map_manager_->isEnabled()) {
    return 0.0;
  }
  if (!risk_map_manager_->isReady()) {
    ROS_WARN_THROTTLE(1.0, "RiskAwareEdge: risk map is not ready; using distance-only cost.");
    return 0.0;
  }

  const int segment_count =
      std::max(1, static_cast<int>(std::ceil(length / params_.sample_resolution)));
  // Reuse the buffer on this thread across edge evaluations.
  thread_local std::vector<double> risk_samples;
  risk_samples.clear();
  risk_samples.reserve(static_cast<std::size_t>(segment_count) + 1);
  double risk_sum = 0.0;
  double risk_max = 0.0;
  for (int sample = 0; sample <= segment_count; ++sample) {
    const double ratio = static_cast<double>(sample) / segment_count;
    const Eigen::Vector2d position =
        ((1.0 - ratio) * start + ratio * end).head<2>();
    const double risk = risk_map_manager_->getRisk(position);
    if (!std::isfinite(risk)) {
      if (maximum_risk != nullptr) {
        *maximum_risk = std::numeric_limits<double>::infinity();
      }
      return std::numeric_limits<double>::infinity();
    }
    risk_samples.push_back(risk);
    risk_sum += risk;
    risk_max = std::max(risk_max, risk);
  }
  if (maximum_risk != nullptr) *maximum_risk = risk_max;
  if (risk_max > std::min(params_.risk_threshold,
                          params_.risk_safe_threshold)) {
    ROS_DEBUG_STREAM("RiskAwareEdge infeasible edge: risk_max=" << risk_max
                     << " risk_safe_threshold=" << params_.risk_safe_threshold);
    return std::numeric_limits<double>::infinity();
  }

  std::sort(risk_samples.begin(), risk_samples.end(), std::greater<double>());
  const std::size_t tail_count = std::max<std::size_t>(
      1, static_cast<std::size_t>(std::ceil(
             params_.risk_cvar_ratio * static_cast<double>(risk_samples.size()))));
  double tail_sum = 0.0;
  for (std::size_t index = 0; index < tail_count; ++index) {
    tail_sum += risk_samples[index];
  }
  const double average_risk = risk_sum / static_cast<double>(risk_samples.size());
  const double cvar_risk = tail_sum / static_cast<double>(tail_count);
  const double combined_risk = average_risk + params_.risk_cvar_weight * cvar_risk;
  ROS_DEBUG_STREAM("RiskAwareEdge sampled risk: average=" << average_risk
                   << " cvar=" << cvar_risk << " max=" << risk_max
                   << " combined=" << combined_risk);
  return combined_risk;
}

double RiskAwareEdge::computeAVCCost(const Eigen::Vector3d& previous,
                                    const Eigen::Vector3d& current,
                                    const Eigen::Vector3d& next) const {
  const Eigen::Vector2d incoming = (current - previous).head<2>();
  const Eigen::Vector2d outgoing = (next - current).head<2>();
  const double incoming_length = incoming.norm();
  const double ds = outgoing.norm();
  if (incoming_length < 1e-6 || ds < 1e-6) {
    return 0.0;
  }

  const Eigen::Vector2d incoming_unit = incoming / incoming_length;
  const Eigen::Vector2d outgoing_unit = outgoing / ds;
  const double dot = std::max(-1.0, std::min(1.0, incoming_unit.dot(outgoing_unit)));
  const double cross = incoming_unit.x() * outgoing_unit.y() -
                       incoming_unit.y() * outgoing_unit.x();
  const double delta_theta = std::abs(std::atan2(cross, dot));
  const double delta_t = ds / params_.nominal_speed;
  const double omega_required = delta_theta / delta_t;
  const double omega_max = params_.nominal_speed / params_.min_turn_radius;
  const double ratio = std::abs(omega_required) / (omega_max + 1e-6);
  if (ratio <= 1.0) return 0.0;
  const double excess = ratio - 1.0;
  return excess * excess;
}

}  // namespace fast_planner
