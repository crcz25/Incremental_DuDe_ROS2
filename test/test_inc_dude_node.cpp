// The image-saving path of the incremental_decomposer node (save trigger).
//
// ROS_handler lives in src/inc_dude.cpp, next to main(), with no header, so
// this test compiles that file with main() renamed. The source is unchanged.
//
// C01_* and C02_* guard the fixes of audit defects C-01 and C-02. C01_*
// detects its defect only in the AddressSanitizer build
// (tools/ci/run_ci.sh asan).

#include <gtest/gtest.h>

// Everything inc_dude.cpp includes that has an include guard, before main is
// renamed (inc_decomp.hpp has no guard and is left to inc_dude.cpp).
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>
#else
#include <cv_bridge/cv_bridge.h>
#endif
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "inc_dude/decomposition_adapter.hpp"
#include "inc_dude/region_msg_conversion.hpp"
#include "inc_dude/region_tracker.hpp"

#define main inc_dude_node_main
#include "../src/inc_dude.cpp"
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
  const fs::path dir = fs::path(::testing::TempDir()) / ("inc_dude_" + name);
  fs::remove_all(dir);
  fs::create_directories(dir);
  return dir;
}

class IncDudeNode : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    output_dir_ = freshDirectory("node_output");
    const std::string param =
        "segmentation_output_directory:=" + output_dir_.string();
    const char *argv[] = {"test_inc_dude_node", "--ros-args", "-p",
                          param.c_str()};
    rclcpp::init(4, argv);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    std::srand(1234);  // the save functions colour regions with rand()
    node_ = std::make_shared<ROS_handler>("map", 3.0f);
  }

  static fs::path output_dir_;
  std::shared_ptr<ROS_handler> node_;
};

fs::path IncDudeNode::output_dir_;

cv::Mat labels(std::initializer_list<uchar> values) {
  cv::Mat m(1, static_cast<int>(values.size()), CV_8UC1);
  int col = 0;
  for (uchar v : values) {
    m.at<uchar>(0, col++) = v;
  }
  return m;
}

}  // namespace

// Batch image: label 0 is the grey background, every label one colour, and
// the colour map returned has one entry per label 0..max plus the grey.
TEST_F(IncDudeNode, SaveImageOriginalColorPaintsLabelsAndReturnsTheColormap) {
  const fs::path path = freshDirectory("original") / "batch.png";
  const std::vector<cv::Vec3b> colormap =
      node_->save_image_original_color(path.string(), labels({0, 1, 2, 2}));

  ASSERT_EQ(colormap.size(), 4u);  // grey + labels 0, 1, 2
  EXPECT_EQ(colormap[0], cv::Vec3b(208, 208, 208));
  const cv::Mat saved = cv::imread(path.string(), cv::IMREAD_COLOR);
  ASSERT_FALSE(saved.empty());
  ASSERT_EQ(saved.size(), cv::Size(4, 1));
  EXPECT_EQ(saved.at<cv::Vec3b>(0, 0), cv::Vec3b(208, 208, 208));
  EXPECT_EQ(saved.at<cv::Vec3b>(0, 1), colormap[1]);
  EXPECT_EQ(saved.at<cv::Vec3b>(0, 2), colormap[2]);
  EXPECT_EQ(saved.at<cv::Vec3b>(0, 3), colormap[2]);
}

// C-01: the colour vector has `max` entries, but labels run 1..max, so the
// largest label is written and read one past its end.
TEST_F(IncDudeNode, C01_SaveDecomposedImageColorHandlesTheLargestLabel) {
  const fs::path path = freshDirectory("decomposed") / "inc.png";
  const std::vector<cv::Vec3b> colormap = {
      {208, 208, 208}, {10, 20, 30}, {40, 50, 60}, {70, 80, 90}};
  const std::map<int, int> inc_to_batch = {{3, 2}};

  node_->save_decomposed_image_color(path.string(), labels({1, 2, 3}),
                                     colormap, inc_to_batch);

  const cv::Mat saved = cv::imread(path.string(), cv::IMREAD_COLOR);
  ASSERT_FALSE(saved.empty());
  // Label 3 (the largest) takes the colour of batch region 2.
  EXPECT_EQ(saved.at<cv::Vec3b>(0, 2), colormap[2]);
}

// C-02: a save trigger before the first map has nothing to save; the
// callback must return without touching the empty images. Before the fix it
// threw cv::Exception from simple_segment() on the empty image, before it
// reached the unchecked test_contour[0]; in the node the exception left the
// subscription callback and stopped the process.
TEST_F(IncDudeNode, C02_SaveTriggerBeforeTheFirstMapIsANoOp) {
  auto trigger = std::make_shared<std_msgs::msg::String>();
  trigger->data = "before_map";
  try {
    node_->chatCallback(trigger);
  } catch (const std::exception &e) {
    ADD_FAILURE() << "chatCallback threw: " << e.what();
  }
  EXPECT_FALSE(fs::exists(output_dir_ / "before_map_Batch.png"));
  EXPECT_FALSE(fs::exists(output_dir_ / "before_map_Inc.png"));
}
