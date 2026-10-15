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
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <list>
#include <sstream>
#include <utility>
#include <optional>
#include <random>
#include <map>
#include <string>
#include <tuple>
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

// H-03: DuDe_OpenCV_wrapper releases what it allocates. One fixed map (two
// rooms joined by a corridor) decomposed 50 times, each by a new wrapper as
// Incremental_Decomposer uses them: every run gives the same contours. In
// the sanitizer build LeakSanitizer reports nothing for this test alone
// (with the DuDe patches in patches/dude/).
void expectRepeatableDecomposition(const cv::Mat &image, size_t min_pieces) {
  std::vector<std::vector<cv::Point>> first;
  for (int run = 0; run < 50; ++run) {
    DuDe_OpenCV_wrapper wrapper;
    wrapper.set_pixel_Tau(20.0f);
    wrapper.Decomposer(image.clone());
    if (run == 0) {
      first = wrapper.Decomposed_contours;
      ASSERT_GE(first.size(), min_pieces);
    } else {
      ASSERT_EQ(wrapper.Decomposed_contours, first) << "run " << run;
    }
  }
}

TEST(DudeWrapper, H03_RepeatedDecompositionsGiveTheSameContours) {
  cv::Mat image = cv::Mat::zeros(120, 160, CV_8UC1);
  cv::rectangle(image, cv::Point(10, 10), cv::Point(70, 110), 255, -1);
  cv::rectangle(image, cv::Point(90, 10), cv::Point(150, 110), 255, -1);
  cv::rectangle(image, cv::Point(70, 50), cv::Point(90, 70), 255, -1);
  expectRepeatableDecomposition(image, 2);  // the two rooms at least
}

// H-03: one room, no cut: the only piece is the wrapper's copy of the input.
TEST(DudeWrapper, H03_ARoomWithoutCutsIsReleasedOnce) {
  cv::Mat image = cv::Mat::zeros(120, 160, CV_8UC1);
  cv::rectangle(image, cv::Point(10, 10), cv::Point(150, 110), 255, -1);
  expectRepeatableDecomposition(image, 1);
}

// P4-N8 (D5): DuDe appends to its global draw_decoration (allFeaturePMs,
// pointers into the decomposed polygons, and allFeatureConcavities) on
// every decomposition. After a wrapper is destroyed the global holds
// nothing, however many decompositions ran.
extern Draw_Decoration draw_decoration;  // DuDe's global (main.h)

TEST(DudeWrapper, D5_TheGlobalDecorationDoesNotGrow) {
  cv::Mat image = cv::Mat::zeros(120, 160, CV_8UC1);
  cv::rectangle(image, cv::Point(10, 10), cv::Point(70, 110), 255, -1);
  cv::rectangle(image, cv::Point(90, 10), cv::Point(150, 110), 255, -1);
  cv::rectangle(image, cv::Point(70, 50), cv::Point(90, 70), 255, -1);
  for (int run = 0; run < 20; ++run) {
    {
      DuDe_OpenCV_wrapper wrapper;
      wrapper.set_pixel_Tau(20.0f);
      wrapper.Decomposer(image.clone());
    }
    ASSERT_EQ(::draw_decoration.allFeaturePMs.size(), 0u) << "run " << run;
    ASSERT_EQ(::draw_decoration.allFeatureConcavities.size(), 0u) << "run " << run;
  }
}

// DuDe reference output: the full result of DuDe for fixed maps (the input
// contour, pocket minima, cut sets, CDT diagonals, cuts, accumulated cuts,
// pieces and contours), every double in hexfloat, compared exactly with
// test/golden/dude_decomposition_reference.txt (recorded before the DuDe
// leak patches in patches/dude/, which must not change it). Each run writes
// what it got to dude_decomposition_reference.actual.txt in the working
// directory (the build directory), never into the source tree.
//
// The result is compared in a canonical form: a segment is an unordered
// pair of end points (the smaller first, by x then y), a closed polygon its
// vertex cycle from its smallest vertex in the direction of the smaller
// neighbour (every vertex kept), and every list is sorted by all printed
// fields. The order of DuDe's lists and the orientation of its segments
// depend on heap addresses (they differ between processes and after other
// decompositions, with or without the patches); the geometry does not.
// These tests run in a process of their own (test_dude_reference).
// The CDT edge hook of patches/dude/ dude-0b, a global of DuDe's cdt2.cpp
// (weak: absent in an unpatched DuDe, where the tests that need it skip).
extern void (*g_cdt_edge_test_hook)(
    std::vector<std::pair<ply_vertex *, ply_vertex *>> &, bool &)
    __attribute__((weak));

namespace dude_reference {

using Pt = std::pair<double, double>;  // x, y
using Seg = std::pair<Pt, Pt>;

struct Cutset {
  bool has_v{false};
  Pt v;
  int type{0};
  int resolves{0};
  std::vector<Seg> diagonals;
};
struct Ply {
  int type{0};
  std::vector<Pt> vertices;  // a closed cycle
};
struct Contour {
  std::vector<Pt> points;  // a closed cycle
  Pt centroid;
};
struct Result {
  std::string name;
  std::vector<Pt> input_contour;  // a closed cycle
  std::vector<Pt> pocket_minima, hole_pocket_minima;
  std::vector<Cutset> cutsets;
  std::vector<Seg> cdt_diagonals, cuts, accumulated_cuts;
  std::vector<std::vector<Ply>> pieces;
  std::vector<Contour> contours;
};

Pt pos(const ply_vertex *v) { return {v->getPos()[0], v->getPos()[1]}; }
Pt pos(const cv::Point &p) { return {double(p.x), double(p.y)}; }

std::vector<Seg> segments(const std::vector<c_diagonal> &diagonals) {
  std::vector<Seg> out;
  for (const c_diagonal &d : diagonals) {
    out.emplace_back(pos(d.getV1()), pos(d.getV2()));
  }
  return out;
}

std::vector<Pt> points(const std::list<ply_vertex *> &vertices) {
  std::vector<Pt> out;
  for (const ply_vertex *v : vertices) {
    out.push_back(pos(v));
  }
  return out;
}

Result decompose(const std::string &name, const cv::Mat &image,
                 float tau_px = 20.0f) {
  DuDe_OpenCV_wrapper wrapper;
  wrapper.set_pixel_Tau(tau_px);
  wrapper.Decomposer(image.clone());
  Result r;
  r.name = name;
  for (const cv::Point &p : wrapper.Parent_contour) {
    r.input_contour.push_back(pos(p));
  }
  r.pocket_minima = points(wrapper.dude.m_PMs);
  r.hole_pocket_minima = points(wrapper.dude.hole_PMs);
  for (const c_cutset &c : wrapper.dude.m_cutsets) {
    Cutset cs;
    cs.has_v = c.getV() != nullptr;
    if (cs.has_v) {
      cs.v = pos(c.getV());
    }
    cs.type = static_cast<int>(c.getType());
    cs.resolves = c.getResolveSize();
    cs.diagonals = segments(c.getDiagonals());
    r.cutsets.push_back(cs);
  }
  r.cdt_diagonals = segments(wrapper.dude.m_diagonals);
  r.cuts = segments(wrapper.dude.m_cuts);
  r.accumulated_cuts = segments(wrapper.draw_decoration.allAccumulatedCuts);
  for (c_polygon *piece : wrapper.finalPolygonPieces) {
    std::vector<Ply> plys;
    for (c_ply &ply : *piece) {
      Ply out;
      out.type = static_cast<int>(ply.getType());
      ply_vertex *v = ply.getHead();
      do {
        out.vertices.push_back(pos(v));
        v = v->getNext();
      } while (v != ply.getHead());
      plys.push_back(out);
    }
    r.pieces.push_back(plys);
  }
  for (size_t i = 0; i < wrapper.Decomposed_contours.size(); ++i) {
    Contour c;
    for (const cv::Point &p : wrapper.Decomposed_contours[i]) {
      c.points.push_back(pos(p));
    }
    c.centroid = pos(wrapper.contours_centroid[i]);
    r.contours.push_back(c);
  }
  return r;
}

// ---- canonical form ---------------------------------------------------------

Seg canonical(Seg s) {
  if (s.second < s.first) {
    std::swap(s.first, s.second);
  }
  return s;
}

std::vector<Seg> canonical(std::vector<Seg> segs) {
  for (Seg &s : segs) {
    s = canonical(s);
  }
  std::sort(segs.begin(), segs.end());
  return segs;
}

// The cycle from its smallest vertex, towards the smaller neighbour; with a
// repeated smallest vertex (or equal neighbours) the smallest of all those
// sequences.
std::vector<Pt> canonicalCycle(const std::vector<Pt> &cycle) {
  if (cycle.empty()) {
    return cycle;
  }
  const size_t n = cycle.size();
  const Pt smallest = *std::min_element(cycle.begin(), cycle.end());
  std::vector<Pt> best;
  for (size_t start = 0; start < n; ++start) {
    if (cycle[start] != smallest) {
      continue;
    }
    for (int step : {1, -1}) {
      std::vector<Pt> seq;
      for (size_t k = 0; k < n; ++k) {
        const long count = static_cast<long>(n);
        long i = (static_cast<long>(start) + step * static_cast<long>(k)) % count;
        seq.push_back(cycle[static_cast<size_t>(i < 0 ? i + count : i)]);
      }
      if (best.empty() || seq < best) {
        best = seq;
      }
    }
  }
  return best;
}

std::vector<Pt> sorted(std::vector<Pt> pts) {
  std::sort(pts.begin(), pts.end());
  return pts;
}

Result canonical(Result r) {
  r.input_contour = canonicalCycle(r.input_contour);
  r.pocket_minima = sorted(r.pocket_minima);
  r.hole_pocket_minima = sorted(r.hole_pocket_minima);
  for (Cutset &c : r.cutsets) {
    c.diagonals = canonical(c.diagonals);
  }
  std::sort(r.cutsets.begin(), r.cutsets.end(), [](const Cutset &a, const Cutset &b) {
    return std::tie(a.has_v, a.v, a.type, a.resolves, a.diagonals) <
           std::tie(b.has_v, b.v, b.type, b.resolves, b.diagonals);
  });
  r.cdt_diagonals = canonical(r.cdt_diagonals);
  r.cuts = canonical(r.cuts);
  r.accumulated_cuts = canonical(r.accumulated_cuts);
  auto plyLess = [](const Ply &a, const Ply &b) {
    return std::tie(a.type, a.vertices) < std::tie(b.type, b.vertices);
  };
  for (std::vector<Ply> &piece : r.pieces) {
    for (Ply &ply : piece) {
      ply.vertices = canonicalCycle(ply.vertices);
    }
    std::sort(piece.begin(), piece.end(), plyLess);
  }
  std::sort(r.pieces.begin(), r.pieces.end(),
            [&](const std::vector<Ply> &a, const std::vector<Ply> &b) {
              return std::lexicographical_compare(a.begin(), a.end(), b.begin(),
                                                  b.end(), plyLess);
            });
  for (Contour &c : r.contours) {
    c.points = canonicalCycle(c.points);
  }
  std::sort(r.contours.begin(), r.contours.end(), [](const Contour &a, const Contour &b) {
    return std::tie(a.points, a.centroid) < std::tie(b.points, b.centroid);
  });
  return r;
}

// ---- text ---------------------------------------------------------------------

void write(std::ostream &out, const Pt &p) {
  out << ' ' << std::hexfloat << p.first << ',' << p.second << std::defaultfloat;
}

void write(std::ostream &out, const std::string &name, const std::vector<Pt> &pts) {
  out << name << ' ' << pts.size() << '\n';
  for (const Pt &p : pts) {
    write(out, p);
  }
  out << '\n';
}

void write(std::ostream &out, const std::string &name, const std::vector<Seg> &segs) {
  out << name << ' ' << segs.size() << '\n';
  for (const Seg &s : segs) {
    write(out, s.first);
    write(out, s.second);
    out << '\n';
  }
}

// `r` as text, in the order and orientation it has.
std::string exactText(const Result &r) {
  std::ostringstream out;
  out << "map " << r.name << '\n';
  write(out, "input_contour", r.input_contour);
  write(out, "pocket_minima", r.pocket_minima);
  write(out, "hole_pocket_minima", r.hole_pocket_minima);
  out << "cutsets " << r.cutsets.size() << '\n';
  for (const Cutset &c : r.cutsets) {
    out << "cutset type " << c.type << " resolves " << c.resolves << " v";
    if (c.has_v) {
      write(out, c.v);
    } else {
      out << " null";
    }
    out << '\n';
    write(out, " cutset_diagonals", c.diagonals);
  }
  write(out, "cdt_diagonals", r.cdt_diagonals);
  write(out, "cuts", r.cuts);
  write(out, "accumulated_cuts", r.accumulated_cuts);
  out << "regions " << r.pieces.size() << '\n';
  for (const std::vector<Ply> &piece : r.pieces) {
    out << "piece plys " << piece.size() << '\n';
    for (const Ply &ply : piece) {
      write(out, " ply type " + std::to_string(ply.type) + " vertices", ply.vertices);
    }
  }
  out << "contours " << r.contours.size() << '\n';
  for (const Contour &c : r.contours) {
    out << " centroid";
    write(out, c.centroid);
    out << '\n';
    write(out, " contour", c.points);
  }
  return out.str();
}

// The canonical form of `r` as text.
std::string text(const Result &raw) { return exactText(canonical(raw)); }

// ---- the maps -----------------------------------------------------------------

std::vector<std::function<Result()>> mapMakers() {
  std::vector<std::function<Result()>> all;
  all.push_back([] {  // a convex room (an octagon)
    cv::Mat image = cv::Mat::zeros(120, 160, CV_8UC1);
    const std::vector<cv::Point> octagon = {{40, 10}, {120, 10}, {150, 40},
                                            {150, 80}, {120, 110}, {40, 110},
                                            {10, 80}, {10, 40}};
    cv::fillConvexPoly(image, octagon, 255);
    return decompose("convex_octagon", image);
  });
  all.push_back([] {  // a room without cuts (H03_ARoomWithoutCutsIsReleasedOnce)
    cv::Mat image = cv::Mat::zeros(120, 160, CV_8UC1);
    cv::rectangle(image, cv::Point(10, 10), cv::Point(150, 110), 255, -1);
    return decompose("room_without_cuts", image);
  });
  all.push_back([] {  // three rooms joined by two corridors (concave, two cuts or more)
    cv::Mat image = cv::Mat::zeros(120, 240, CV_8UC1);
    cv::rectangle(image, cv::Point(10, 10), cv::Point(70, 110), 255, -1);
    cv::rectangle(image, cv::Point(90, 10), cv::Point(150, 110), 255, -1);
    cv::rectangle(image, cv::Point(170, 10), cv::Point(230, 110), 255, -1);
    cv::rectangle(image, cv::Point(70, 50), cv::Point(90, 70), 255, -1);
    cv::rectangle(image, cv::Point(150, 30), cv::Point(170, 45), 255, -1);
    return decompose("three_rooms", image);
  });
  all.push_back([] {  // a room with a hole (an obstacle in the middle)
    cv::Mat image = cv::Mat::zeros(160, 200, CV_8UC1);
    cv::rectangle(image, cv::Point(10, 10), cv::Point(190, 150), 255, -1);
    cv::rectangle(image, cv::Point(80, 60), cv::Point(120, 100), 0, -1);
    return decompose("room_with_hole", image);
  });
  all.push_back([] {  // a comb: a corridor with six teeth (many cuts)
    cv::Mat image = cv::Mat::zeros(130, 250, CV_8UC1);
    cv::rectangle(image, cv::Point(10, 10), cv::Point(240, 40), 255, -1);
    for (int x = 14; x + 16 <= 240; x += 40) {
      cv::rectangle(image, cv::Point(x, 40), cv::Point(x + 16, 110), 255, -1);
    }
    return decompose("comb", image);
  });
  all.push_back([] {  // a square spiral corridor (many diagonal candidates)
    cv::Mat image = cv::Mat::zeros(200, 200, CV_8UC1);
    const std::vector<cv::Point> path = {{20, 20},  {180, 20}, {180, 180},
                                         {40, 180}, {40, 50},  {150, 50},
                                         {150, 150}, {70, 150}, {70, 80},
                                         {120, 80}, {120, 120}};
    cv::polylines(image, path, false, 255, 12);
    return decompose("spiral", image);
  });
  all.push_back([] {  // nine rooms on a 3 x 3 grid joined by corridors
    cv::Mat image = cv::Mat::zeros(200, 260, CV_8UC1);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        const cv::Point o(10 + 85 * c, 10 + 62 * r);
        cv::rectangle(image, o, o + cv::Point(60, 40), 255, -1);
        if (c < 2) cv::rectangle(image, o + cv::Point(60, 14), o + cv::Point(85, 26), 255, -1);
        if (r < 2) cv::rectangle(image, o + cv::Point(22, 40), o + cv::Point(36, 62), 255, -1);
      }
    }
    return decompose("room_grid", image);
  });
  all.push_back([] {  // a real map: the free cells of Freiburg52_scan, opened (5 x 5) like
     // the node's cleaning removes thin noise. At tau 20 px DuDe crashes on
     // this map in c_dude::is_bridge_breaking (unpatched DuDe as well).
    const std::filesystem::path map =
        std::filesystem::path(__FILE__).parent_path().parent_path() /
        "maps/Room_Segmentation/all_maps/Freiburg52_scan.png";
    const cv::Mat gray = cv::imread(map.string(), cv::IMREAD_GRAYSCALE);
    if (gray.empty()) {
      throw std::runtime_error("cannot read " + map.string());
    }
    cv::Mat free_space = gray > 250;
    cv::morphologyEx(free_space, free_space, cv::MORPH_OPEN,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)));
    // tau 2.7 m at 0.05 m per pixel, the evaluation setting (54 px)
    return decompose("Freiburg52_scan", free_space, 54.0f);
  });
  return all;
}

using CdtEdge = std::pair<ply_vertex *, ply_vertex *>;
struct EdgeOrder {
  std::mt19937 rng;
  bool sort{true};
  size_t equal_edges{0};  // edges equal on all four coordinates
};
EdgeOrder &edgeOrder() {
  static EdgeOrder order;
  return order;
}
void shuffleEdges(std::vector<CdtEdge> &edges, bool &sort) {
  std::vector<CdtEdge> sorted_copy = edges;
  std::sort(sorted_copy.begin(), sorted_copy.end(), [](const CdtEdge &a, const CdtEdge &b) {
    return std::make_tuple(a.first->getPos()[0], a.first->getPos()[1],
                           a.second->getPos()[0], a.second->getPos()[1]) <
           std::make_tuple(b.first->getPos()[0], b.first->getPos()[1],
                           b.second->getPos()[0], b.second->getPos()[1]);
  });
  for (size_t i = 1; i < sorted_copy.size(); ++i) {
    if (sorted_copy[i - 1].first->getPos() == sorted_copy[i].first->getPos() &&
        sorted_copy[i - 1].second->getPos() == sorted_copy[i].second->getPos()) {
      ++edgeOrder().equal_edges;
    }
  }
  std::shuffle(edges.begin(), edges.end(), edgeOrder().rng);
  sort = edgeOrder().sort;
}
// Installs shuffleEdges with `seed` for its lifetime.
struct ShuffledEdges {
  ShuffledEdges(unsigned seed, bool sort) {
    edgeOrder().rng.seed(seed);
    edgeOrder().sort = sort;
    ::g_cdt_edge_test_hook = &shuffleEdges;
  }
  ~ShuffledEdges() { ::g_cdt_edge_test_hook = nullptr; }
};

std::vector<Result> allMaps() {
  std::vector<Result> all;
  for (const auto &make : mapMakers()) {
    all.push_back(make());
  }
  return all;
}

std::string exactText(const std::vector<Result> &all) {
  std::string out;
  for (const Result &r : all) {
    out += exactText(r);
  }
  return out;
}

// Hooks for checking the exact result under other histories (unset in the
// normal run): DUDE_REFERENCE_WARMUP=2,0,4 decomposes those maps first;
// DUDE_REFERENCE_PREALLOC=n allocates n blocks of varied sizes and frees
// every other one, so later allocations land elsewhere.
std::vector<std::unique_ptr<char[]>> &preallocated() {
  static std::vector<std::unique_ptr<char[]>> blocks;
  return blocks;
}
void applyHistoryHooks() {
  if (const char *n = std::getenv("DUDE_REFERENCE_PREALLOC")) {
    const int count = std::atoi(n);
    for (int i = 0; i < count; ++i) {
      preallocated().emplace_back(new char[16 + (i * 37) % 4096]);
    }
    for (size_t i = 0; i < preallocated().size(); i += 2) {
      preallocated()[i].reset();
    }
  }
  if (const char *w = std::getenv("DUDE_REFERENCE_WARMUP")) {
    const auto makers = mapMakers();
    std::stringstream order(w);
    std::string index;
    while (std::getline(order, index, ',')) {
      makers.at(static_cast<size_t>(std::stoi(index)))();
    }
  }
}

std::string text(const std::vector<Result> &all) {
  std::string out;
  for (const Result &r : all) {
    out += text(r);
  }
  return out;
}

const Result &byName(const std::vector<Result> &all, const std::string &name) {
  for (const Result &r : all) {
    if (r.name == name) {
      return r;
    }
  }
  throw std::runtime_error("no map " + name);
}

}  // namespace dude_reference

TEST(DudeReference, TheDecompositionOfFixedMapsIsUnchanged) {
  const std::string actual = dude_reference::text(dude_reference::allMaps());
  std::ofstream("dude_decomposition_reference.actual.txt") << actual;
  const std::filesystem::path golden =
      std::filesystem::path(__FILE__).parent_path() /
      "golden/dude_decomposition_reference.txt";
  std::ifstream in(golden);
  ASSERT_TRUE(in) << "missing " << golden;
  std::stringstream expected;
  expected << in.rdbuf();
  EXPECT_TRUE(expected.str() == actual)
      << "differs from " << golden
      << "; see dude_decomposition_reference.actual.txt";
}

// The exact result (DuDe's order and orientation; recorded with the stable
// order of patches/dude/ dude-0, without the leak fixes) is unchanged, also
// after other decompositions (DUDE_REFERENCE_WARMUP) or allocations
// (DUDE_REFERENCE_PREALLOC).
TEST(DudeReference, TheExactDecompositionIsUnchanged) {
  dude_reference::applyHistoryHooks();
  const std::string actual = dude_reference::exactText(dude_reference::allMaps());
  std::ofstream("dude_decomposition_exact_reference.actual.txt") << actual;
  const std::filesystem::path golden =
      std::filesystem::path(__FILE__).parent_path() /
      "golden/dude_decomposition_exact_reference.txt";
  std::ifstream in(golden);
  ASSERT_TRUE(in) << "missing " << golden;
  std::stringstream expected;
  expected << in.rdbuf();
  EXPECT_TRUE(expected.str() == actual)
      << "differs from " << golden
      << "; see dude_decomposition_exact_reference.actual.txt";
}

// A second pass in the same process (after the first decompositions) gives
// the same canonical result.
TEST(DudeReference, ASecondPassGivesTheSameResult) {
  const auto first = dude_reference::allMaps();
  const auto second = dude_reference::allMaps();
  EXPECT_TRUE(dude_reference::text(second) == dude_reference::text(first));
  EXPECT_TRUE(dude_reference::exactText(second) == dude_reference::exactText(first));
}

TEST(DudeReference, TheMapsCoverCutsAndAHole) {
  const auto all = dude_reference::allMaps();
  EXPECT_GE(dude_reference::byName(all, "three_rooms").cuts.size(), 2u);
  EXPECT_FALSE(dude_reference::byName(all, "room_with_hole").hole_pocket_minima.empty());
  for (const auto &r : all) {
    EXPECT_FALSE(r.pieces.empty()) << r.name;
  }
}

// Negative control: the canonical comparison still sees a vertex moved by
// one ulp and a missing cut, and ignores a pure reorder of the pieces.
TEST(DudeReference, TheComparisonDetectsRealChanges) {
  const dude_reference::Result r =
      dude_reference::byName(dude_reference::allMaps(), "three_rooms");
  const std::string reference = dude_reference::text(r);
  ASSERT_GE(r.pieces.size(), 2u);
  ASSERT_FALSE(r.cuts.empty());

  dude_reference::Result moved = r;
  double &x = moved.pieces[0][0].vertices[0].first;
  x = std::nextafter(x, HUGE_VAL);
  EXPECT_NE(dude_reference::text(moved), reference);

  dude_reference::Result missing_cut = r;
  missing_cut.cuts.pop_back();
  EXPECT_NE(dude_reference::text(missing_cut), reference);

  dude_reference::Result reordered = r;
  std::swap(reordered.pieces[0], reordered.pieces[1]);
  std::reverse(reordered.cuts.begin(), reordered.cuts.end());
  std::swap(reordered.cuts[0].first, reordered.cuts[0].second);
  EXPECT_EQ(dude_reference::text(reordered), reference);
}

// dude-0b: DuDe's exact result does not depend on the order in which CGAL
// returns the CDT edges: 20 fixed shuffles of every edge list (the sort
// kept) give the same exact dump for every map.
TEST(DudeReference, TheExactResultDoesNotDependOnTheCdtEdgeOrder) {
  if (&::g_cdt_edge_test_hook == nullptr) {
    GTEST_SKIP() << "DuDe without patches/dude/ dude-0b";
  }
  const std::string expected = dude_reference::exactText(dude_reference::allMaps());
  for (unsigned seed = 1; seed <= 20; ++seed) {
    dude_reference::ShuffledEdges shuffled(seed, true);
    EXPECT_TRUE(dude_reference::exactText(dude_reference::allMaps()) == expected)
        << "seed " << seed;
  }
  std::cout << "CDT edges equal on all four coordinates: "
            << dude_reference::edgeOrder().equal_edges << std::endl;
}

// dude-0b: without the sort, the same 20 shuffles of the CDT edges give the
// same geometry (canonical dump) for every map: DuDe's choice of diagonals
// does not depend on their order for these maps.
TEST(DudeReference, TheGeometryDoesNotDependOnTheUnsortedCdtEdgeOrder) {
  if (&::g_cdt_edge_test_hook == nullptr) {
    GTEST_SKIP() << "DuDe without patches/dude/ dude-0b";
  }
  const std::string expected = dude_reference::text(dude_reference::allMaps());
  for (unsigned seed = 1; seed <= 20; ++seed) {
    dude_reference::ShuffledEdges shuffled(seed, false);
    EXPECT_TRUE(dude_reference::text(dude_reference::allMaps()) == expected)
        << "seed " << seed;
  }
}
