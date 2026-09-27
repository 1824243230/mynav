#include <gtest/gtest.h>

#include <path_searching/risk_aware_path_selector.h>

#include <limits>

namespace fast_planner {
namespace {

std::vector<PathSelectionCandidate> makeReliabilityCandidates() {
  PathSelectionCandidate low_reliability;
  low_reliability.path = {Eigen::Vector3d(0.0, 0.0, 0.0),
                          Eigen::Vector3d(1.0, 0.0, 0.0)};
  low_reliability.length = 1.0;
  low_reliability.risk = 1.0;
  low_reliability.prs_score = 0.1;

  PathSelectionCandidate high_reliability = low_reliability;
  high_reliability.prs_score = 0.9;
  return {low_reliability, high_reliability};
}

TEST(PathReliabilityRankingTest, RewardsHigherPrsWhenEnabled) {
  RiskAwarePathSelector::Parameters parameters;
  parameters.reliability_enabled = true;
  parameters.lambda_length = 1.0;
  parameters.lambda_risk = 1.0;
  parameters.lambda_prs = 1.0;
  parameters.orientation_tie_threshold = 0.0;
  RiskAwarePathSelector selector(parameters);

  const PathSelectionResult result =
      selector.selectBestPath(makeReliabilityCandidates(), 0.0);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.best_index, 1U);
  EXPECT_NEAR(result.prs_score, 0.9, 1e-12);
  EXPECT_TRUE(result.reliability_enabled);
}

TEST(PathReliabilityRankingTest, IgnoresPrsAndUsesLegacyRankingWhenDisabled) {
  RiskAwarePathSelector::Parameters parameters;
  parameters.reliability_enabled = false;
  parameters.orientation_tie_threshold = 0.0;
  RiskAwarePathSelector selector(parameters);

  const PathSelectionResult result =
      selector.selectBestPath(makeReliabilityCandidates(), 0.0);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.best_index, 0U);
  EXPECT_DOUBLE_EQ(result.prs_score, 0.0);
  EXPECT_FALSE(result.reliability_enabled);
}

TEST(PathReliabilityRankingTest, SkipsInfeasibleRisk) {
  RiskAwarePathSelector selector;
  auto candidates = makeReliabilityCandidates();
  candidates[0].risk = std::numeric_limits<double>::infinity();

  const PathSelectionResult result = selector.selectBestPath(candidates, 0.0);

  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.best_index, 1U);
}

TEST(PathReliabilityRankingTest, ReportsFailureWhenAllRisksAreInfeasible) {
  RiskAwarePathSelector selector;
  auto candidates = makeReliabilityCandidates();
  for (auto& candidate : candidates) {
    candidate.risk = std::numeric_limits<double>::infinity();
  }

  EXPECT_FALSE(selector.selectBestPath(candidates, 0.0).success);
}

TEST(TopologyStability, KeepsCurrentUntilGainExceedsThreshold) {
  ros::Time::init();
  RiskAwarePathSelector selector;
  auto candidates = makeReliabilityCandidates();
  candidates[0].topology_id = 11;
  candidates[0].path_cost = 1.0;
  candidates[1].topology_id = 22;
  candidates[1].path_cost = 0.85;

  EXPECT_EQ(selector.selectStablePath(candidates).best_index, 1U);
  EXPECT_EQ(selector.currentTopologyId(), 0U);
  selector.commitTopology(11);
  const PathSelectionResult keep = selector.selectStablePath(candidates);
  EXPECT_EQ(keep.best_index, 0U);
  EXPECT_STREQ(keep.switch_decision, "KEEP");
  EXPECT_NEAR(keep.gain, 0.05, 1e-9);

  candidates[1].path_cost = 0.8;
  const PathSelectionResult boundary = selector.selectStablePath(candidates);
  EXPECT_EQ(boundary.best_index, 0U);
  EXPECT_STREQ(boundary.switch_decision, "KEEP");
  EXPECT_LE(boundary.gain, 0.1);

  candidates[1].path_cost = 0.7;
  const PathSelectionResult change = selector.selectStablePath(candidates);
  EXPECT_EQ(change.best_index, 1U);
  EXPECT_STREQ(change.switch_decision, "SWITCH");
  EXPECT_NEAR(change.gain, 0.2, 1e-9);
  EXPECT_EQ(selector.currentTopologyId(), 11U);
}

TEST(TopologyStability, ForcesSwitchWhenCurrentCandidateIsMissing) {
  ros::Time::init();
  RiskAwarePathSelector selector;
  selector.commitTopology(11);
  auto candidates = makeReliabilityCandidates();
  candidates[0].topology_id = 22;
  candidates[0].path_cost = 2.0;
  candidates[1].topology_id = 33;
  candidates[1].path_cost = 1.0;

  const PathSelectionResult result = selector.selectStablePath(candidates);

  EXPECT_EQ(result.best_index, 1U);
  EXPECT_STREQ(result.switch_decision, "FORCE_SWITCH");
  selector.commitTopology(33);
  EXPECT_EQ(selector.currentTopologyId(), 33U);
  selector.resetTopology();
  EXPECT_EQ(selector.currentTopologyId(), 0U);
}

}  // namespace
}  // namespace fast_planner
