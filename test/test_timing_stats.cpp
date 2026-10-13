// L-05: TimingStats keeps a fixed amount of memory however many durations
// it records; count, sum, minimum and maximum stay exact and the median
// estimate stays within the stated limit.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>

#include "inc_dude/timing_stats.hpp"

namespace {

using inc_dude::TimingStats;

TEST(TimingStats, L05_FixedMemoryExactTotalsBoundedMedianError) {
  TimingStats s;
  const size_t capacity = s.sampleCapacity();
  const uint64_t n = 10 * TimingStats::kSampleSize;  // 10240
  double sum = 0.0;
  for (uint64_t i = 0; i < n; ++i) {
    // Every value 1..n once, in a fixed scrambled order.
    const double v = static_cast<double>((i * 7919) % n + 1);
    s.add(v);
    sum += v;
  }
  EXPECT_EQ(s.count(), n);
  EXPECT_DOUBLE_EQ(s.sum(), sum);
  EXPECT_DOUBLE_EQ(s.min(), 1.0);
  EXPECT_DOUBLE_EQ(s.max(), static_cast<double>(n));
  EXPECT_EQ(s.sampleSize(), TimingStats::kSampleSize);
  EXPECT_EQ(s.sampleCapacity(), capacity);  // never grew
  // The true median is n / 2; the estimate is within 5 % of the ranks.
  EXPECT_NEAR(s.quantile(0.5), n / 2.0, 0.05 * n);
}

TEST(TimingStats, L05_ExactWhileTheSampleHoldsEveryValue) {
  TimingStats s;
  EXPECT_EQ(s.quantile(0.5), 0.0);
  for (int v : {5, 1, 4, 2, 3}) {
    s.add(v);
  }
  EXPECT_DOUBLE_EQ(s.quantile(0.5), 3.0);  // sorted[5 / 2]
  EXPECT_DOUBLE_EQ(s.mean(), 3.0);
  EXPECT_DOUBLE_EQ(s.sumSquares(), 55.0);
}

}  // namespace
