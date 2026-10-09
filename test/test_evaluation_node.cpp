// The evaluation node (src/evaluation.cpp): saving and segmenting the
// dataset images.
//
// ROS_handler lives in src/evaluation.cpp, next to main(), with no header,
// so this test compiles that file with main() renamed. The source is
// unchanged. It is a separate executable from test_inc_dude_node because
// both files define a class ROS_handler.

#include <gtest/gtest.h>

// Everything evaluation.cpp includes that has an include guard, before main
// is renamed (inc_decomp.hpp has no guard and is left to evaluation.cpp).
#include <dirent.h>

#include <chrono>
#include <functional>
#include <memory>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

#define main evaluation_node_main
#include "../src/evaluation.cpp"  // NOLINT(bugprone-suspicious-include): the test seam
#undef main

#include <cstdlib>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

// A directory of its own per test, under the build tree's temp directory.
fs::path freshDirectory(const std::string &name) {
  fs::path dir = fs::path(::testing::TempDir()) / ("evaluation_" + name);
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

// The node's dataset_path: an empty directory unless a test adds images.
const fs::path &datasetDir() {
  static const fs::path dir = freshDirectory("dataset");
  return dir;
}

class EvaluationNode : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    const std::string path = "dataset_path:=" + datasetDir().string();
    const char *argv[] = {"test_evaluation_node", "--ros-args", "-p",
                          path.c_str()};
    rclcpp::init(4, argv);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    std::srand(1234);  // the save functions colour regions with rand()
    node_ = std::make_shared<ROS_handler>(2.7f);
  }

  std::shared_ptr<ROS_handler> node_;
};

cv::Mat labels(std::initializer_list<uchar> values) {
  cv::Mat m(1, static_cast<int>(values.size()), CV_8UC1);
  int col = 0;
  for (uchar v : values) {
    m.at<uchar>(0, col++) = v;
  }
  return m;
}

}  // namespace

// C-01 (evaluation.cpp copy): the colour vector has `max` entries, but labels
// run 1..max; the largest label must get its colour without writing or
// reading past the vector (detected by the AddressSanitizer build).
TEST_F(EvaluationNode, C01_SaveDecomposedImageColorHandlesTheLargestLabel) {
  const fs::path path = freshDirectory("decomposed") / "inc.png";
  const std::vector<cv::Vec3b> colormap = {
      {208, 208, 208}, {10, 20, 30}, {40, 50, 60}, {70, 80, 90}};
  const std::map<int, int> inc_to_batch = {{3, 2}};

  node_->save_decomposed_image_color(path.string(), labels({1, 2, 3}),
                                     colormap, inc_to_batch);

  const cv::Mat saved = cv::imread(path.string(), cv::IMREAD_COLOR);
  ASSERT_FALSE(saved.empty());
  EXPECT_EQ(saved.at<cv::Vec3b>(0, 2), colormap[2]);
}
