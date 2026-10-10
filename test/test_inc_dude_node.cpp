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
#include "std_srvs/srv/trigger.hpp"

#define main inc_dude_node_main
#include "../src/inc_dude.cpp"
#undef main

#include <cstdlib>
#include <filesystem>
#include <optional>
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
  std::string message;
  try {
    EXPECT_FALSE(node_->saveSegmentation("before_map", message));
  } catch (const std::exception &e) {
    ADD_FAILURE() << "saveSegmentation threw: " << e.what();
  }
  EXPECT_FALSE(fs::exists(output_dir_ / "before_map_Batch.png"));
  EXPECT_FALSE(fs::exists(output_dir_ / "before_map_Inc.png"));
}

namespace {

// Two rooms joined by a door (free 0, walls 100, unknown -1 around them),
// 120 x 160 cells of 0.05 m: a map the decomposer splits into regions.
nav_msgs::msg::OccupancyGrid::SharedPtr twoRoomMap() {
  auto map = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map->header.frame_id = "map";
  map->info.resolution = 0.05f;
  map->info.width = 160;
  map->info.height = 120;
  map->info.origin.orientation.w = 1.0;
  map->data.assign(std::size_t{160} * 120, -1);
  auto cell = [&](int r, int c) -> int8_t & { return map->data[r * 160 + c]; };
  for (int r = 15; r < 105; ++r) {
    for (int c = 15; c < 145; ++c) {
      const bool border = r < 20 || r >= 100 || c < 20 || c >= 140;
      const bool wall = c >= 78 && c < 82 && !(r >= 50 && r < 70);
      cell(r, c) = border || wall ? 100 : 0;
    }
  }
  return map;
}

}  // namespace

// H-01: the save names the files; a name that leaves the output directory
// must not write anything, a plain name saves both images.
TEST_F(IncDudeNode, H01_SaveTriggerNamesCannotLeaveTheOutputDirectory) {
  node_->mapCallback(twoRoomMap());
  const fs::path outside = output_dir_.parent_path();
  fs::remove(outside / "h01_escaped_Batch.png");
  fs::remove(outside / "h01_escaped_Inc.png");

  std::string message;
  for (const std::string name : {"../h01_escaped", "/tmp/h01_escaped",
                                 "a/../../h01_escaped", ".."}) {
    EXPECT_FALSE(node_->saveSegmentation(name, message)) << name;
  }
  EXPECT_FALSE(fs::exists(outside / "h01_escaped_Batch.png"));
  EXPECT_FALSE(fs::exists(outside / "h01_escaped_Inc.png"));
  EXPECT_FALSE(fs::exists("/tmp/h01_escaped_Batch.png"));

  EXPECT_TRUE(node_->saveSegmentation("h01_valid", message)) << message;
  EXPECT_TRUE(fs::exists(output_dir_ / "h01_valid_Batch.png"));
  EXPECT_TRUE(fs::exists(output_dir_ / "h01_valid_Inc.png"));
}

namespace {

using std_srvs::srv::Trigger;

// Calls the node's private ~/save service (bounded waits for discovery and
// the response; nothing is timed).
std::optional<Trigger::Response> callSave(const std::shared_ptr<ROS_handler> &node) {
  auto client_node = std::make_shared<rclcpp::Node>("h02_client");
  auto client =
      client_node->create_client<Trigger>("/incremental_decomposer/save");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(client_node);
  for (int i = 0; i < 300 && !client->service_is_ready(); ++i) {
    executor.spin_some(std::chrono::milliseconds(10));
  }
  if (!client->service_is_ready()) {
    return std::nullopt;
  }
  auto future = client->async_send_request(std::make_shared<Trigger::Request>());
  if (executor.spin_until_future_complete(future, std::chrono::seconds(30)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    return std::nullopt;
  }
  return *future.get();
}

}  // namespace

// H-02: the save trigger is the node's private ~/save service; it names the
// files with the save_file_name parameter.
TEST_F(IncDudeNode, H02_SaveServiceBeforeTheFirstMapFails) {
  const auto response = callSave(node_);
  ASSERT_TRUE(response) << "no ~/save service";
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("no map"), std::string::npos) << response->message;
}

TEST_F(IncDudeNode, H02_SaveServiceSavesTheNamedImagesAfterAMap) {
  node_->mapCallback(twoRoomMap());
  node_->set_parameter(rclcpp::Parameter("save_file_name", "h02_saved"));
  const auto response = callSave(node_);
  ASSERT_TRUE(response) << "no ~/save service";
  EXPECT_TRUE(response->success) << response->message;
  EXPECT_TRUE(fs::exists(output_dir_ / "h02_saved_Batch.png"));
  EXPECT_TRUE(fs::exists(output_dir_ / "h02_saved_Inc.png"));
}

TEST_F(IncDudeNode, H02_SaveServiceRejectsAnInvalidName) {
  node_->mapCallback(twoRoomMap());
  const fs::path outside = output_dir_.parent_path();
  fs::remove(outside / "h02_escaped_Batch.png");
  node_->set_parameter(rclcpp::Parameter("save_file_name", "../h02_escaped"));
  const auto response = callSave(node_);
  ASSERT_TRUE(response) << "no ~/save service";
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("file name"), std::string::npos) << response->message;
  EXPECT_FALSE(fs::exists(outside / "h02_escaped_Batch.png"));
}

TEST_F(IncDudeNode, H02_SaveServiceReportsAWriteFailure) {
  node_->mapCallback(twoRoomMap());
  // A directory where the image should go: the write fails.
  fs::create_directories(output_dir_ / "h02_blocked_Batch.png");
  node_->set_parameter(rclcpp::Parameter("save_file_name", "h02_blocked"));
  const auto response = callSave(node_);
  ASSERT_TRUE(response) << "no ~/save service";
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("write"), std::string::npos) << response->message;
  // Without its Batch image, no Inc image is written either.
  EXPECT_FALSE(fs::exists(output_dir_ / "h02_blocked_Inc.png"));
}

// H-02: a message on the shared 'chatter' topic no longer saves anything.
TEST_F(IncDudeNode, H02_ChatterNoLongerTriggersASave) {
  node_->mapCallback(twoRoomMap());
  auto source = std::make_shared<rclcpp::Node>("h02_chatter_source");
  auto publisher = source->create_publisher<std_msgs::msg::String>("chatter", 1);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  executor.add_node(source);
  for (int i = 0; i < 100 && publisher->get_subscription_count() == 0; ++i) {
    executor.spin_some(std::chrono::milliseconds(10));
  }
  std_msgs::msg::String trigger;
  trigger.data = "h02_chatter";
  publisher->publish(trigger);
  for (int i = 0; i < 100; ++i) {
    executor.spin_some(std::chrono::milliseconds(10));
  }
  EXPECT_FALSE(fs::exists(output_dir_ / "h02_chatter_Batch.png"));
  EXPECT_FALSE(fs::exists(output_dir_ / "h02_chatter_Inc.png"));
}
