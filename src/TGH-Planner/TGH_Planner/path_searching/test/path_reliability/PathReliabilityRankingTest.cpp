#include <gtest/gtest.h>

#include <path_searching/risk_aware_path_selector.h>

#include <cmath>

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

TEST(PathReliabilityRankingTest, FixedScaleCostDoesNotDependOnCandidateExtrema) {
  RiskAwarePathSelector::Parameters parameters;
  parameters.ects_enabled = true;
  parameters.use_fixed_scale_score = true;
  parameters.reliability_enabled = false;
  parameters.w1 = 1.0;
  parameters.w2 = 1.0;
  parameters.length_ref = 10.0;
  parameters.risk_ref = 5.0;
  parameters.orientation_tie_threshold = 0.0;
  RiskAwarePathSelector selector(parameters);

  PathSelectionCandidate first;
  first.path = {Eigen::Vector3d(0.0, 0.0, 0.0),
                Eigen::Vector3d(1.0, 0.0, 0.0)};
  first.length = 2.0;
  first.risk = 4.0;
  PathSelectionCandidate second = first;
  second.length = 3.0;
  second.risk = 1.0;

  const PathSelectionResult base = selector.selectBestPath({first, second}, 0.0);
  PathSelectionCandidate outlier = first;
  outlier.length = 1000.0;
  outlier.risk = 1000.0;
  const PathSelectionResult extended =
      selector.selectBestPath({first, second, outlier}, 0.0);

  ASSERT_TRUE(base.success);
  ASSERT_TRUE(extended.success);
  ASSERT_EQ(base.candidate_scores.size(), 2U);
  ASSERT_EQ(extended.candidate_scores.size(), 3U);
  EXPECT_TRUE(base.fixed_scale_score);
  EXPECT_EQ(base.best_index, 1U);
  EXPECT_EQ(extended.best_index, 1U);
  EXPECT_NEAR(base.candidate_scores[0].route_cost,
              extended.candidate_scores[0].route_cost, 1e-12);
  EXPECT_NEAR(base.candidate_scores[1].route_cost,
              extended.candidate_scores[1].route_cost, 1e-12);
}

TEST(PathReliabilityRankingTest, FixedScaleCostIncludesConfiguredPrsTerm) {
  RiskAwarePathSelector::Parameters parameters;
  parameters.ects_enabled = true;
  parameters.use_fixed_scale_score = true;
  parameters.reliability_enabled = true;
  parameters.lambda_length = 1.0;
  parameters.lambda_risk = 2.0;
  parameters.lambda_prs = 3.0;
  parameters.length_ref = 2.0;
  parameters.risk_ref = 4.0;
  parameters.prs_ref = 0.5;
  RiskAwarePathSelector selector(parameters);

  PathSelectionCandidate candidate;
  candidate.path = {Eigen::Vector3d(0.0, 0.0, 0.0),
                    Eigen::Vector3d(1.0, 0.0, 0.0)};
  candidate.length = 2.0;
  candidate.risk = 4.0;
  candidate.prs_score = 0.5;

  const PathSelectionResult result = selector.selectBestPath({candidate}, 0.0);

  ASSERT_TRUE(result.success);
  ASSERT_EQ(result.candidate_scores.size(), 1U);
  EXPECT_NEAR(result.route_cost, 6.0, 1e-12);
  EXPECT_NEAR(result.candidate_scores[0].normalized_prs, 1.0, 1e-12);
}

TEST(PathReliabilityRankingTest, SwitchingPenaltyKeepsHardToReachChallenger) {
  RiskAwarePathSelector::Parameters parameters;
  parameters.lambda_switch = 1.0;
  parameters.switch_margin = 0.05;
  parameters.dubins_ref = 1.0;
  RiskAwarePathSelector selector(parameters);

  const TopologySwitchDecision decision =
      selector.evaluateTopologySwitch(10.0, 9.0, 1.0, 3.0, false);

  EXPECT_NEAR(decision.gain, 0.1, 1e-6);
  EXPECT_NEAR(decision.connection_penalty, 2.0, 1e-12);
  EXPECT_LT(decision.margin, parameters.switch_margin);
  EXPECT_FALSE(decision.propose_switch);
}

TEST(PathReliabilityRankingTest, SwitchingMarginEmitsProposal) {
  RiskAwarePathSelector::Parameters parameters;
  parameters.lambda_switch = 0.5;
  parameters.switch_margin = 0.1;
  parameters.dubins_ref = 2.0;
  RiskAwarePathSelector selector(parameters);

  const TopologySwitchDecision decision =
      selector.evaluateTopologySwitch(10.0, 7.0, 1.0, 1.4, false);

  EXPECT_NEAR(decision.connection_penalty, 0.2, 1e-12);
  EXPECT_NEAR(decision.margin, 0.2, 1e-6);
  EXPECT_TRUE(decision.propose_switch);
}

TEST(PathReliabilityRankingTest, InvalidActiveTopologyIgnoresConnectionPenalty) {
  RiskAwarePathSelector::Parameters parameters;
  parameters.lambda_switch = 10.0;
  parameters.switch_margin = 0.05;
  parameters.dubins_ref = 1.0;
  RiskAwarePathSelector selector(parameters);

  const TopologySwitchDecision decision =
      selector.evaluateTopologySwitch(10.0, 8.0, 1.0, 100.0, true);

  EXPECT_TRUE(decision.active_topology_invalid);
  EXPECT_DOUBLE_EQ(decision.connection_penalty, 0.0);
  EXPECT_NEAR(decision.margin, decision.gain, 1e-12);
  EXPECT_TRUE(decision.propose_switch);
}

TEST(PathReliabilityRankingTest, NonPositiveGainAlwaysKeepsTopology) {
  RiskAwarePathSelector selector;

  const TopologySwitchDecision decision =
      selector.evaluateTopologySwitch(10.0, 10.5, 1.0, 1.0, false);

  EXPECT_LE(decision.gain, 0.0);
  EXPECT_FALSE(decision.propose_switch);
}

TEST(PathReliabilityRankingTest, MissingChallengerConnectionKeepsTopology) {
  RiskAwarePathSelector selector;

  const TopologySwitchDecision decision = selector.evaluateTopologySwitch(
      10.0, 5.0, 1.0, std::numeric_limits<double>::infinity(), false);

  EXPECT_TRUE(std::isinf(decision.connection_penalty));
  EXPECT_FALSE(decision.propose_switch);
}

}  // namespace
}  // namespace fast_planner
