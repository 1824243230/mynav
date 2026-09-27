#include <path_searching/risk_aware_path_selector.h>

#include <gtest/gtest.h>

#include <limits>

namespace fast_planner {
namespace {

TEST(TCBSScoreTest, InvalidRiskScaleFallsBackToFixedDefault) {
  RiskAwarePathSelector::Parameters parameters;
  parameters.high_risk_threshold =
      std::numeric_limits<double>::quiet_NaN();
  RiskAwarePathSelector selector(parameters);
  const std::vector<Eigen::Vector3d> path = {
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(1.0, 0.0, 0.0)};

  const TCBSScore score = selector.evaluateTCBSScore(path, 1.0, 2.5);
  EXPECT_TRUE(score.feasible);
  EXPECT_NEAR(score.efficiency, 0.0, 1e-8);
  EXPECT_NEAR(score.risk, 0.5, 1e-8);
  EXPECT_NEAR(score.bottleneck, 0.5, 1e-8);
}

TEST(TCBSScoreTest, NonFinitePathIsInfeasible) {
  RiskAwarePathSelector selector;
  const std::vector<Eigen::Vector3d> path = {
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(std::numeric_limits<double>::infinity(), 0.0, 0.0)};

  EXPECT_FALSE(selector.evaluateTCBSScore(path, 1.0, 0.0).feasible);
}

}  // namespace
}  // namespace fast_planner
