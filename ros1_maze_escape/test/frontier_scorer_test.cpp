#include <limits>

#include <gtest/gtest.h>

#include "ros1_maze_escape/frontier_scorer.hpp"

namespace ros1_maze_escape {
namespace {

HeuristicFrontierScorer makeDefaultScorer() {
  return HeuristicFrontierScorer(HeuristicWeights{});
}

TEST(FrontierScorerTest, ComputesConfiguredWeightedScore) {
  const HeuristicFrontierScorer scorer = makeDefaultScorer();
  FrontierFeatures features;
  features.information_gain = 1.0;
  features.actual_path_length = 2.0;
  features.clearance = 3.0;
  features.heading_change = 0.5;
  features.visit_count = 1.0;
  features.failure_count = 1.0;
  features.exit_probability = 0.2;

  const FrontierScore result = scorer.score(features);

  ASSERT_TRUE(result.valid);
  EXPECT_NEAR(result.value, -4.1, 1e-12);
  EXPECT_EQ(result.source, "heuristic");
}

TEST(FrontierScorerTest, LongerPathHasLowerScore) {
  const HeuristicFrontierScorer scorer = makeDefaultScorer();
  FrontierFeatures short_path;
  short_path.actual_path_length = 1.0;
  FrontierFeatures long_path = short_path;
  long_path.actual_path_length = 3.0;

  const FrontierScore short_result = scorer.score(short_path);
  const FrontierScore long_result = scorer.score(long_path);

  ASSERT_TRUE(short_result.valid);
  ASSERT_TRUE(long_result.valid);
  EXPECT_GT(short_result.value, long_result.value);
}

TEST(FrontierScorerTest, GreaterClearanceHasHigherScore) {
  const HeuristicFrontierScorer scorer = makeDefaultScorer();
  FrontierFeatures narrow;
  narrow.clearance = 0.1;
  FrontierFeatures clear = narrow;
  clear.clearance = 0.8;

  const FrontierScore narrow_result = scorer.score(narrow);
  const FrontierScore clear_result = scorer.score(clear);

  ASSERT_TRUE(narrow_result.valid);
  ASSERT_TRUE(clear_result.valid);
  EXPECT_GT(clear_result.value, narrow_result.value);
}

TEST(FrontierScorerTest, MoreFailuresHaveLowerScore) {
  const HeuristicFrontierScorer scorer = makeDefaultScorer();
  FrontierFeatures no_failures;
  FrontierFeatures repeated_failures;
  repeated_failures.failure_count = 2.0;

  const FrontierScore clean_result = scorer.score(no_failures);
  const FrontierScore failed_result = scorer.score(repeated_failures);

  ASSERT_TRUE(clean_result.valid);
  ASSERT_TRUE(failed_result.valid);
  EXPECT_GT(clean_result.value, failed_result.value);
}

TEST(FrontierScorerTest, RejectsNanFeature) {
  const HeuristicFrontierScorer scorer = makeDefaultScorer();
  FrontierFeatures features;
  features.clearance = std::numeric_limits<double>::quiet_NaN();

  const FrontierScore result = scorer.score(features);

  EXPECT_FALSE(result.valid);
}

TEST(FrontierScorerTest, RejectsInfiniteFeature) {
  const HeuristicFrontierScorer scorer = makeDefaultScorer();
  FrontierFeatures features;
  features.actual_path_length = std::numeric_limits<double>::infinity();

  const FrontierScore result = scorer.score(features);

  EXPECT_FALSE(result.valid);
}

}  // namespace
}  // namespace ros1_maze_escape

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
