#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <random>
#include <vector>

namespace inc_dude {

// L-05: running statistics of a series of durations in fixed memory. The
// count, sum, sum of squares, minimum and maximum are exact. Quantiles come
// from a uniform random sample of at most kSampleSize values (reservoir
// sampling, fixed seed): exact while count() <= kSampleSize, an estimate
// after that (the sample's rank error is about 1 / sqrt(kSampleSize), 3 %,
// at one standard deviation).
class TimingStats
{
public:
  static constexpr size_t kSampleSize = 1024;

  TimingStats() {sample_.reserve(kSampleSize);}

  void add(double value)
  {
    ++count_;
    sum_ += value;
    sum_squares_ += value * value;
    min_ = count_ == 1 ? value : std::min(min_, value);
    max_ = count_ == 1 ? value : std::max(max_, value);
    if (sample_.size() < kSampleSize) {
      sample_.push_back(value);
    } else {
      const uint64_t slot = rng_() % count_;
      if (slot < kSampleSize) {
        sample_[slot] = value;
      }
    }
  }

  uint64_t count() const {return count_;}
  double sum() const {return sum_;}
  double sumSquares() const {return sum_squares_;}
  double min() const {return min_;}  // 0 while empty
  double max() const {return max_;}  // 0 while empty
  double mean() const {return count_ ? sum_ / static_cast<double>(count_) : 0.0;}

  // The sample value at index floor(q * size) of the sorted sample (q in
  // [0, 1]; q = 0.5: the upper median, as sorted[size / 2]); 0 while empty.
  double quantile(double q) const
  {
    if (sample_.empty()) {
      return 0.0;
    }
    std::vector<double> sorted = sample_;
    std::sort(sorted.begin(), sorted.end());
    const auto index = static_cast<size_t>(q * static_cast<double>(sorted.size()));
    return sorted[std::min(index, sorted.size() - 1)];
  }

  size_t sampleSize() const {return sample_.size();}
  size_t sampleCapacity() const {return sample_.capacity();}

private:
  uint64_t count_ = 0;
  double sum_ = 0.0;
  double sum_squares_ = 0.0;
  double min_ = 0.0;
  double max_ = 0.0;
  std::vector<double> sample_;
  std::mt19937_64 rng_{20261013};
};

}  // namespace inc_dude
