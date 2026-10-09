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
#include <unistd.h>
#include <vector>

#include <sensor_msgs/msg/image.hpp>

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

// The topic the node publishes its third (no-furniture) image on.
const std::string &thirdTopic() {
  static const std::string topic =
      "/test_evaluation_third_" + std::to_string(::getpid());
  return topic;
}

// Two rooms joined by a door (free = 255), 120 x 160 pixels; with
// `furniture`, a black block inside the left room.
cv::Mat roomImage(bool furniture) {
  cv::Mat m = cv::Mat::zeros(120, 160, CV_8UC1);
  m(cv::Rect(20, 20, 120, 80)).setTo(255);
  m(cv::Rect(78, 20, 4, 80)).setTo(0);   // the wall between the rooms
  m(cv::Rect(78, 50, 4, 20)).setTo(255);  // its door
  if (furniture) {
    m(cv::Rect(35, 35, 15, 15)).setTo(0);
  }
  return m;
}
constexpr int kFurnitureRow = 42, kFurnitureCol = 42;

// Writes the ground truth, furniture and no-furniture images of `name`.
void writeDataset(const std::string &name) {
  fs::create_directories(datasetDir() / "Tagged_Images");
  cv::imwrite((datasetDir() / (name + "_gt_segmentation.png")).string(),
              roomImage(false));
  cv::imwrite((datasetDir() / (name + "_furnitures.png")).string(),
              roomImage(true));
  cv::imwrite((datasetDir() / (name + ".png")).string(), roomImage(false));
}

class EvaluationNode : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    const std::string path = "dataset_path:=" + datasetDir().string();
    const std::string third = "inc_dude_segmentation_topic:=" + thirdTopic();
    const char *argv[] = {"test_evaluation_node", "--ros-args", "-p",
                          path.c_str(), "-p", third.c_str()};
    rclcpp::init(6, argv);
  }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override {
    std::srand(1234);  // the save functions colour regions with rand()
    node_ = std::make_shared<ROS_handler>(2.7f);
  }

  // The image the node publishes on its third topic (publish_Image()).
  sensor_msgs::msg::Image::SharedPtr publishedThirdImage() {
    auto listener = std::make_shared<rclcpp::Node>("evaluation_listener");
    sensor_msgs::msg::Image::SharedPtr received;
    auto sub = listener->create_subscription<sensor_msgs::msg::Image>(
        thirdTopic(), rclcpp::QoS(1).reliable(),
        [&](sensor_msgs::msg::Image::SharedPtr m) { received = std::move(m); });
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(listener);
    // Bounded waits for discovery and delivery (no timing is asserted).
    for (int i = 0; i < 500 && sub->get_publisher_count() == 0; ++i) {
      executor.spin_some(std::chrono::milliseconds(10));
    }
    node_->publish_Image();
    for (int i = 0; i < 500 && !received; ++i) {
      executor.spin_some(std::chrono::milliseconds(10));
    }
    return received;
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

// H-12: a ground truth without free space has no contour; the drawing loop
// must not index the empty hierarchy.
TEST_F(EvaluationNode, H12_GroundTruthWithoutFreeSpaceGivesAnEmptySegmentation) {
  const cv::Mat segmented = node_->segment_Ground_Truth(cv::Mat::zeros(50, 60, CV_8UC1));
  ASSERT_EQ(segmented.size(), cv::Size(60, 50));
  EXPECT_EQ(cv::countNonZero(segmented), 0);
}

// H-12: a missing input file is reported and skipped, not processed.
TEST_F(EvaluationNode, H12_MissingInputFilesAreSkipped) {
  try {
    node_->process_files_twice("missing");
    node_->process_files("missing");
    node_->process_files_incrementally("missing");
  } catch (const std::exception &e) {
    ADD_FAILURE() << "processing a missing file threw: " << e.what();
  }
  EXPECT_FALSE(fs::exists(datasetDir() / "Tagged_Images" / "missing_TAG_gt_segmentation.png"));
}

// H-12: the third topic carries the no-furniture image, not the furniture
// one (process_files, which reads all three images).
TEST_F(EvaluationNode, H12_ThirdTopicCarriesTheNoFurnitureImage) {
  writeDataset("rooms");
  node_->process_files("rooms");
  const auto third = publishedThirdImage();
  ASSERT_TRUE(third);
  const cv::Mat image = cv_bridge::toCvCopy(third)->image;
  ASSERT_EQ(image.size(), cv::Size(160, 120));
  EXPECT_FLOAT_EQ(image.at<float>(kFurnitureRow, kFurnitureCol), 255.0f);
}

// H-12, audit line 509: process_files_twice reads no no-furniture image;
// its third topic must not carry the furniture image.
TEST_F(EvaluationNode, H12_ProcessFilesTwiceDoesNotPublishFurnitureAsNoFurniture) {
  writeDataset("rooms_twice");
  node_->process_files_twice("rooms_twice");
  const auto third = publishedThirdImage();
  ASSERT_TRUE(third);
  const cv::Mat image = cv_bridge::toCvCopy(third)->image;
  EXPECT_TRUE(image.empty()) << image.size();
}
