#include <path_searching/risk_aware_edge.h>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

namespace fast_planner {
namespace {

TEST(RiskMapUnknownExploration, UnknownIsTraversableButOccupiedIsNot) {
  ros::Time::init();
  RiskMapManager::Parameters parameters;
  parameters.risk_enable = true;
  parameters.lambda_distance = 1.0;
  parameters.lambda_corridor = 0.5;
  parameters.lambda_unknown = 0.5;
  parameters.robot_width = 0.5;
  RiskMapManager manager(parameters);
  std::vector<char> occupancy(100, -1);
  std::vector<double> esdf(100, 0.0);
  occupancy[5 + 5 * 10] = 0;
  esdf[5 + 5 * 10] = 1.0;
  occupancy[9 + 5 * 10] = 1;

  ASSERT_TRUE(manager.updateRiskMap(occupancy, esdf, 10, 10, 0.1,
                                    Eigen::Vector2d::Zero(), "world"));
  EXPECT_TRUE(std::isfinite(manager.getRisk(0, 0)));
  EXPECT_LT(manager.getRisk(0, 0), 15.0);
  EXPECT_GT(manager.getRisk(0, 0), parameters.lambda_unknown);
  EXPECT_GT(manager.getCorridorWidth(5, 5), parameters.robot_width);
  EXPECT_LT(manager.getRisk(5, 5), 15.0);
  EXPECT_TRUE(std::isinf(manager.getRisk(9, 5)));
}

TEST(RiskAwareEdgeNormalization, UsesCandidateSetMaxima) {
  RiskAwareEdge evaluator;
  std::vector<RiskPathCost> costs(2);
  costs[0].length = 10.0;
  costs[0].risk = 1.0;
  costs[0].total_cost = 11.0;
  costs[1].length = 5.0;
  costs[1].risk = 4.0;
  costs[1].total_cost = 9.0;

  evaluator.normalizeCandidateCosts(costs);

  EXPECT_NEAR(costs[0].total_cost, 1.25, 1e-9);
  EXPECT_NEAR(costs[1].total_cost, 1.5, 1e-9);
  EXPECT_LT(costs[0].total_cost, costs[1].total_cost);
}

TEST(RiskAwareEdgeNormalization, HandlesZeroAndInvalidMaxima) {
  RiskAwareEdge evaluator;
  std::vector<RiskPathCost> costs(3);
  costs[0].total_cost = 0.0;
  costs[1].risk = std::numeric_limits<double>::infinity();
  costs[1].total_cost = std::numeric_limits<double>::infinity();
  costs[2].length = 1.0;
  costs[2].maximum_risk = evaluator.getParameters().risk_safe_threshold + 1.0;
  costs[2].total_cost = 1.0;

  evaluator.normalizeCandidateCosts(costs);

  EXPECT_DOUBLE_EQ(costs[0].total_cost, 0.0);
  EXPECT_TRUE(std::isinf(costs[1].total_cost));
  EXPECT_TRUE(std::isinf(costs[2].total_cost));
}

TEST(RiskAwareEdgeAVC, PenalizesTightTurnAndAveragesOverEdges) {
  RiskAwareEdge evaluator;
  const Eigen::Vector3d previous(0.0, 0.0, 8.0);
  const Eigen::Vector3d current(1.0, 0.0, -4.0);
  const Eigen::Vector3d next(1.0, 1.0, 2.0);
  const double ratio = (std::acos(-1.0) / 2.0) / (1.0 + 1e-6);
  const double expected_avc = (ratio - 1.0) * (ratio - 1.0);

  const RiskEdge edge = evaluator.evaluateEdge(1, 2, current, next, &previous);
  const RiskPathCost path = evaluator.evaluatePath({previous, current, next});

  EXPECT_NEAR(edge.avc_cost, expected_avc, 1e-9);
  EXPECT_DOUBLE_EQ(edge.curvature_cost, edge.avc_cost);
  EXPECT_NEAR(path.avc_cost, expected_avc / 2.0, 1e-9);
  EXPECT_DOUBLE_EQ(path.curvature_cost, path.avc_cost);
}

TEST(RiskAwareEdgeAVC, AllowsGentleAndDegenerateTurns) {
  RiskAwareEdge evaluator;
  const Eigen::Vector3d previous(0.0, 0.0, 0.0);
  const Eigen::Vector3d current(1.0, 0.0, 0.0);
  const Eigen::Vector3d next(1.0, 2.0, 0.0);

  EXPECT_DOUBLE_EQ(evaluator.evaluateEdge(1, 2, current, next, &previous).avc_cost,
                   0.0);
  EXPECT_DOUBLE_EQ(evaluator.evaluateEdge(1, 2, current, current, &previous).avc_cost,
                   0.0);
}

TEST(RiskAwareEdgeTwoStage, RefinesOnlyTopK) {
  ros::Time::init();
  RiskAwareEdge evaluator;
  std::vector<std::vector<Eigen::Vector3d>> paths;
  for (int length = 1; length <= 12; ++length) {
    paths.push_back({Eigen::Vector3d::Zero(),
                     Eigen::Vector3d(static_cast<double>(length), 0.0, 0.0)});
  }
  std::vector<std::size_t> selected_indices;
  std::vector<RiskPathCost> fine_costs;

  evaluator.evaluateCandidatePaths(paths, selected_indices, fine_costs);

  ASSERT_EQ(selected_indices.size(), 10u);
  ASSERT_EQ(fine_costs.size(), 10u);
  for (std::size_t index = 0; index < selected_indices.size(); ++index) {
    EXPECT_EQ(selected_indices[index], index);
    EXPECT_EQ(fine_costs[index].edges.size(), 1u);
  }
}

TEST(RiskAwareEdgeTwoStage, RefinesAllWhenBelowTopK) {
  ros::Time::init();
  RiskAwareEdge evaluator;
  const std::vector<std::vector<Eigen::Vector3d>> paths = {
      {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(2.0, 0.0, 0.0)},
      {Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector3d(1.0, 0.0, 0.0)}};
  std::vector<std::size_t> selected_indices;
  std::vector<RiskPathCost> fine_costs;

  evaluator.evaluateCandidatePaths(paths, selected_indices, fine_costs);

  ASSERT_EQ(selected_indices.size(), paths.size());
  ASSERT_EQ(fine_costs.size(), paths.size());
  EXPECT_EQ(selected_indices[0], 1u);
  EXPECT_EQ(selected_indices[1], 0u);
}

TEST(RiskAwareEdgeTwoStage, RejectsPathsWithoutAnEdge) {
  ros::Time::init();
  RiskAwareEdge evaluator;
  const std::vector<std::vector<Eigen::Vector3d>> paths = {
      {}, {Eigen::Vector3d::Zero()},
      {Eigen::Vector3d::Zero(), Eigen::Vector3d(1.0, 0.0, 0.0)}};
  std::vector<std::size_t> selected_indices;
  std::vector<RiskPathCost> fine_costs;

  evaluator.evaluateCandidatePaths(paths, selected_indices, fine_costs);

  ASSERT_EQ(selected_indices.size(), 1u);
  EXPECT_EQ(selected_indices[0], 2u);
  EXPECT_TRUE(std::isinf(evaluator.evaluatePath(paths[0]).total_cost));
  EXPECT_TRUE(std::isinf(evaluator.evaluatePath(paths[1]).total_cost));
}

}  // namespace
}  // namespace fast_planner
