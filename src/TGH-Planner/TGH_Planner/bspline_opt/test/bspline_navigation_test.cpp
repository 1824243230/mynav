#include <gtest/gtest.h>
#include <bspline_opt/bspline_optimizer.h>
#include <limits>

namespace fast_planner {
class BsplineOptimizerTestAccess {
public:
  static double yawCost(BsplineOptimizer& optimizer,
                        const std::vector<Eigen::Vector3d>& points,
                        std::vector<Eigen::Vector3d>& gradient, double interval) {
    optimizer.bspline_interval_ = interval;
    optimizer.min_r_ = 2.0;
    optimizer.stop_signal_ = false;
    optimizer.if_printf_ = false;
    optimizer.x_var_vec_.assign(points.size(), 0);
    gradient.resize(points.size());
    double cost;
    optimizer.calcCtrlPtYawCost(points, cost, gradient);
    return cost;
  }
};

TEST(BsplineNavigation, TurningGradientMatchesFiniteDifferences) {
  BsplineOptimizer optimizer;
  std::vector<Eigen::Vector3d> points{{0, 0, 0}, {0.3, 0.1, 0},
      {0.4, 0.5, 0}, {0.8, 0.6, 0}, {0.9, 1.0, 0}};
  std::vector<Eigen::Vector3d> gradient, scratch;
  ASSERT_GT(BsplineOptimizerTestAccess::yawCost(optimizer, points, gradient, 0.2), 0.0);
  const double step = 1e-6;
  for (size_t i = 0; i < points.size(); ++i) {
    for (int axis = 0; axis < 3; ++axis) {
      points[i][axis] += step;
      const double plus = BsplineOptimizerTestAccess::yawCost(optimizer, points, scratch, 0.2);
      points[i][axis] -= 2 * step;
      const double minus = BsplineOptimizerTestAccess::yawCost(optimizer, points, scratch, 0.2);
      points[i][axis] += step;
      EXPECT_NEAR(gradient[i][axis], (plus - minus) / (2 * step), 1e-4);
    }
  }
}

TEST(BsplineNavigation, TurningCostUsesCurrentIntervalAndHandlesRest) {
  BsplineOptimizer optimizer;
  std::vector<Eigen::Vector3d> points{{0, 0, 0}, {0, 0, 0},
      {0.3, 0, 0}, {0.4, 0.4, 0}, {0.7, 0.5, 0}};
  std::vector<Eigen::Vector3d> gradient;
  const double first = BsplineOptimizerTestAccess::yawCost(optimizer, points, gradient, 0.1);
  const double second = BsplineOptimizerTestAccess::yawCost(optimizer, points, gradient, 0.2);
  EXPECT_NEAR(first, 4.0 * second, 1e-8);
  for (const auto& value : gradient) EXPECT_TRUE(value.allFinite());
}

TEST(BsplineNavigation, TimeReallocationConvergesFromRest) {
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(14, 3);
  for (int i = 3; i < 14; ++i) points(i, 0) = 0.35 * (i - 2);
  NonUniformBspline trajectory(points, 3, 0.1);
  trajectory.setPhysicalLimits(2.1, 1.65);
  ASSERT_FALSE(trajectory.checkFeasibility());
  for (int i = 0; i < 3; ++i) trajectory.reallocateTime();
  EXPECT_FALSE(trajectory.checkFeasibility());
  for (int i = 3; i < 30 && !trajectory.checkFeasibility(); ++i)
    trajectory.reallocateTime();
  EXPECT_TRUE(trajectory.checkFeasibility(true));
  EXPECT_TRUE(trajectory.getKnot().allFinite());
}

TEST(BsplineNavigation, RejectsNonFiniteControlPoints) {
  Eigen::MatrixXd points = Eigen::MatrixXd::Zero(8, 3);
  points(3, 0) = std::numeric_limits<double>::quiet_NaN();
  NonUniformBspline trajectory(points, 3, 0.1);
  trajectory.setPhysicalLimits(2.1, 1.65);
  EXPECT_FALSE(trajectory.checkFeasibility());
}
}  // namespace fast_planner

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
