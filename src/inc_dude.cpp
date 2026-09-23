// ROS
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

// openCV
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp>  // Iron and later
#else
#include <cv_bridge/cv_bridge.h>  // Humble
#endif
#include <image_transport/image_transport.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/image_encodings.hpp>

// DuDe
#include "inc_decomp.hpp"

// Region tracking
#include "inc_dude/decomposition_adapter.hpp"
#include "inc_dude/region_msg_conversion.hpp"
#include "inc_dude/region_tracker.hpp"

#include <chrono>
#include <functional>
#include <cmath>
#include <memory>
#include <stdexcept>

class ROS_handler : public rclcpp::Node {

  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv_bridge::CvImagePtr cv_ptr;

  std::string mapname_;
  std::string maps_path_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr chat_sub_;
  rclcpp::TimerBase::SharedPtr timer;

  float Decomp_threshold_;
  Incremental_Decomposer inc_decomp;
  Stable_graph Stable;

  cv::Mat image2save_clean, image2save_black, image2save_Inc;

  std::vector<double> clean_time_vector, decomp_time_vector, paint_time_vector,
      complete_time_vector;

  std::string segmentation_output_directory_;
  double offline_image_resolution_;
  int valid_space_threshold_;
  int free_space_threshold_;
  int occupied_min_threshold_;
  int occupied_max_threshold_;
  int obstacle_filter_size_;
  int free_space_filter_size_;
  int obstacle_dilation_iterations_;
  int offline_free_threshold_;

  // Region tracking
  inc_dude::AdapterConfig adapter_config_;
  std::unique_ptr<inc_dude::RegionTracker> tracker_;
  bool publish_missing_regions_;
  bool debug_region_tracking_;
  rclcpp::Publisher<inc_dude::msg::Region2DArray>::SharedPtr regions_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      region_markers_pub_;
  // Grid placement the incremental decomposer's pixel-space state refers to.
  bool have_grid_{false};
  std::string grid_frame_id_;
  inc_dude::GridGeometry grid_;

public:
  ROS_handler(const std::string &mapname, float threshold)
      : Node("incremental_decomposer"), mapname_(mapname),
        Decomp_threshold_(
            this->declare_parameter<double>("decomp_threshold", threshold)) {
    const std::string default_maps_path =
        ament_index_cpp::get_package_share_directory("inc_dude") + "/maps";
    maps_path_ = this->declare_parameter<std::string>("maps_path",
                                                      default_maps_path);
    if (maps_path_.empty()) {
      maps_path_ = default_maps_path;
      this->set_parameter(rclcpp::Parameter("maps_path", maps_path_));
    }
    segmentation_output_directory_ = this->declare_parameter<std::string>(
        "segmentation_output_directory",
        maps_path_ + "/Topological_Segmentation");
    if (segmentation_output_directory_.empty()) {
      segmentation_output_directory_ = maps_path_ + "/Topological_Segmentation";
      this->set_parameter(rclcpp::Parameter("segmentation_output_directory",
                                            segmentation_output_directory_));
    }
    const std::string map_topic =
        this->declare_parameter<std::string>("map_topic", mapname_);
    const std::string save_trigger_topic =
        this->declare_parameter<std::string>("save_trigger_topic", "chatter");
    const std::string tagged_image_topic = this->declare_parameter<std::string>(
        "tagged_image_topic", "/tagged_image");
    const int publish_period_ms =
        this->declare_parameter<int>("publish_period_ms", 500);
    valid_space_threshold_ =
        this->declare_parameter<int>("valid_space_threshold", 101);
    free_space_threshold_ =
        this->declare_parameter<int>("free_space_threshold", 10);
    occupied_min_threshold_ =
        this->declare_parameter<int>("occupied_min_threshold", 90);
    occupied_max_threshold_ =
        this->declare_parameter<int>("occupied_max_threshold", 100);
    obstacle_filter_size_ =
        this->declare_parameter<int>("obstacle_filter_size", 2);
    free_space_filter_size_ =
        this->declare_parameter<int>("free_space_filter_size", 10);
    obstacle_dilation_iterations_ =
        this->declare_parameter<int>("obstacle_dilation_iterations", 4);
    offline_image_resolution_ = this->declare_parameter<double>(
        "offline_image_resolution", 0.05);
    offline_free_threshold_ =
        this->declare_parameter<int>("offline_free_threshold", 250);

    declare_region_tracking_parameters();

    RCLCPP_INFO(this->get_logger(), "Waiting for the map");
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic, rclcpp::QoS(2).transient_local().reliable(),
        std::bind(&ROS_handler::mapCallback, this, std::placeholders::_1));
    chat_sub_ = this->create_subscription<std_msgs::msg::String>(
        save_trigger_topic, 1,
        std::bind(&ROS_handler::chatCallback, this, std::placeholders::_1));
    timer = this->create_wall_timer(
        std::chrono::milliseconds(publish_period_ms),
        std::bind(&ROS_handler::metronomeCallback, this));

    image_pub_ = image_transport::create_publisher(this, tagged_image_topic);

    const std::string regions_topic = this->declare_parameter<std::string>(
        "regions_topic", "/inc_dude/regions");
    // Latched so late subscribers (e.g. the 3DSG layer) get the last state.
    regions_pub_ = this->create_publisher<inc_dude::msg::Region2DArray>(
        regions_topic, rclcpp::QoS(1).transient_local().reliable());
    const std::string markers_topic = this->declare_parameter<std::string>(
        "region_markers_topic", "/inc_dude/region_markers");
    if (debug_region_tracking_) {
      region_markers_pub_ =
          this->create_publisher<visualization_msgs::msg::MarkerArray>(
              markers_topic, rclcpp::QoS(1).transient_local().reliable());
    }
    cv_ptr.reset(new cv_bridge::CvImage);
    cv_ptr->encoding = "mono8";
  }

  /////////////////////////////
  // ROS CALLBACKS
  ////////////////////////////////

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    double begin_process, end_process, begin_whole, occupancy_time,
        decompose_time, drawPublish_time, whole_time;
    begin_whole = begin_process = getTime();

    RCLCPP_INFO(this->get_logger(), "Received a %u X %u map @ %.3f m/pix",
                map->info.width, map->info.height, map->info.resolution);

    inc_dude::GridGeometry grid;
    std::string invalid_reason;
    if (!inc_dude::gridGeometryFromMap(*map, grid, invalid_reason)) {
      RCLCPP_WARN(this->get_logger(), "Ignoring invalid map: %s",
                  invalid_reason.c_str());
      return;
    }
    if (!check_grid_consistency(map->header.frame_id, grid)) {
      return;
    }

    ///////////////////////Occupancy to clean image
    cv::Mat grad, img(map->info.height, map->info.width, CV_8U);
    img.data = (unsigned char *)(&(map->data[0]));
    cv::Mat received_image = img.clone();

    float pixel_Tau = Decomp_threshold_ / map->info.resolution;
    cv_ptr->header = map->header;
    cv::Point2f origin =
        cv::Point2f(map->info.origin.position.x, map->info.origin.position.y);

    cv::Rect first_rect = find_image_bounding_Rect(received_image);
    if (first_rect.empty()) {
      RCLCPP_WARN(this->get_logger(),
                  "Map has no known cells yet, skipping decomposition");
      return;
    }
    float rect_area = (first_rect.height) * (first_rect.width);
    float img_area = (received_image.rows) * (received_image.cols);
    cout << "Area Ratio " << (rect_area / img_area) * 100 << "% " << endl;

    cv::Mat cropped_img;
    received_image(first_rect)
        .copyTo(cropped_img); /////////// Cut the relevant image

    cv::Mat image_cleaned = cv::Mat::zeros(received_image.size(), CV_8UC1);
    cv::Mat black_image = cv::Mat::zeros(received_image.size(), CV_8UC1);

    //*
    cv::Mat black_image2,
        image_cleaned2 = clean_image2(cropped_img, black_image2);

    image_cleaned2.copyTo(image_cleaned(first_rect));
    black_image2.copyTo(black_image(first_rect));
    //*/
    //			image_cleaned = clean_image2(received_image,
    //black_image); 			image2save_clean = image_cleaned.clone();
    cv::flip(image_cleaned, image2save_clean, 0);

    end_process = getTime();
    occupancy_time = end_process - begin_process;

    //			Incremental_Decomposer inc_decomp_batch; //Uncoment to
    //batch

    ///////////////////////// Decompose Image
    begin_process = getTime();

    bool decomposition_ok = false;
    try {
      Stable = inc_decomp.decompose_image(image_cleaned, pixel_Tau, origin,
                                          map->info.resolution);
      //				Stable =
      //inc_decomp_batch.decompose_image(image_cleaned, pixel_Tau, origin,
      //map->info.resolution); //Uncoment to batch
      decomposition_ok = true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Decomposition failed: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(this->get_logger(), "Decomposition failed");
    }

    if (decomposition_ok) {
      track_and_publish_regions(map->header, grid);
    } else if (debug_region_tracking_) {
      RCLCPP_INFO(this->get_logger(),
                  "[region tracking] update skipped: decomposition failed");
    }

    end_process = getTime();
    decompose_time = end_process - begin_process;

    ////////////Draw Image & publish

    begin_process = getTime();
    /*
            //		cv::Mat croppedRef(Colored_Frontier, resize_rect);
                            cv::flip(black_image, black_image,0);  cv::Mat big =
    Stable.draw_stable_contour() & ~black_image;

                            cout << "Rect "<< first_rect << endl;

                            big(first_rect).copyTo(grad);

    //*/
    cv::flip(black_image, black_image, 0);
    image2save_black = black_image.clone();
    grad = Stable.draw_stable_contour() & ~black_image;
    //			grad = image_cleaned;

    image2save_Inc = grad.clone();

    cv_ptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    grad.convertTo(grad, CV_32F);
    //			cv_ptr->encoding =
    //sensor_msgs::image_encodings::TYPE_8UC1;
    //grad.convertTo(grad, CV_8UC1);
    grad.copyTo(cv_ptr->image); ////most important

    end_process = getTime();
    drawPublish_time = end_process - begin_process;
    whole_time = end_process - begin_whole;

    /////// Time Measures
    {
      printf(
          "Time: total %.0f, Classified: occ %.1f, Decomp %.1f, Draw %.1f \n",
          whole_time, occupancy_time, decompose_time, drawPublish_time);

      clean_time_vector.push_back(occupancy_time);
      decomp_time_vector.push_back(decompose_time);
      paint_time_vector.push_back(drawPublish_time);
      complete_time_vector.push_back(whole_time);

      cout << "Time Vector size " << clean_time_vector.size() << endl;
    }

    //*
    double cum_time = 0, cum_quad_time = 0;
    for (int i = 0; i < clean_time_vector.size(); i++) {
      //				cout << time_vector[i] << endl;
      printf("%.0f %.0f %.0f %.0f \n", paint_time_vector[i],
             clean_time_vector[i], decomp_time_vector[i],
             complete_time_vector[i]);
      cum_time += complete_time_vector[i];
      cum_quad_time += complete_time_vector[i] * complete_time_vector[i];
    }
    float avg_time = cum_time / clean_time_vector.size();
    float avg_quad_time = cum_quad_time / clean_time_vector.size();
    float std_time = sqrt(avg_quad_time - avg_time * avg_time);

    std::cout << "Number of Regions " << Stable.Region_contour.size()
              << std::endl;

    //			std::cout << clean_time_vector.size();
    //			printf(" frames processed. Avg time: %.0f + %.0f ms \n",
    //avg_time, std_time);

    //			cv::flip(black_image, black_image,0);  cv::Mat big =
    //Stable.draw_stable_contour() & ~black_image; 			save_images_color(big);
    //*/

    /////////////////////////
  }

  /////////////////
  void metronomeCallback() {
    //		  RCLCPP_INFO(this->get_logger(), "tic tac");
    publish_Image();
  }

  ////////////////
  void chatCallback(const std_msgs::msg::String::SharedPtr chat_msg) {
    std::cout << "chat in" << std::endl;

    std::string saving_path = segmentation_output_directory_ + "/";
    cv::Mat proxy, zero = cv::Mat::zeros(image2save_clean.size(), CV_8U);
    ///////////
    cv::Mat Batch_segmentated = simple_segment(image2save_clean);
    //////////
    std::map<int, int> Batch_Inc_map =
        compare_images(Batch_segmentated, image2save_Inc);

    Batch_segmentated.copyTo(proxy, ~image2save_black);
    Batch_segmentated = proxy.clone();

    double min, max_batch, max_inc;
    cv::minMaxLoc(Batch_segmentated, &min, &max_batch);
    cv::minMaxLoc(image2save_Inc, &min, &max_inc);

    cv::Mat destroyable_batch = Batch_segmentated.clone();
    std::vector<std::vector<cv::Point>> test_contour;
    cv::findContours(destroyable_batch, test_contour, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    cv::Rect first_rect = cv::boundingRect(test_contour[0]);
    for (int i = 1; i < test_contour.size(); i++) {
      first_rect |= cv::boundingRect(test_contour[i]);
    }

    cv::Mat cropped_Batch, cropped_Inc;
    /*
                            cv::Mat image_roi = Batch_segmentated(first_rect);
                            image_roi.copyTo(cropped_Batch);


                            float rect_area =
       (first_rect.height)*(first_rect.width); float img_area =
       (Batch_segmentated.rows) * (Batch_segmentated.cols); cout <<"Area Ratio "
       <<  ( rect_area/img_area  )*100 <<"% "<< endl;
                            */
    Batch_segmentated(first_rect)
        .copyTo(cropped_Batch); /////////// Cut the relevant image
    image2save_Inc(first_rect)
        .copyTo(cropped_Inc); /////////// Cut the relevant image

    if (true) {
      std::vector<cv::Vec3b> colormap = save_image_original_color(
          saving_path + chat_msg->data + "_Batch.png", cropped_Batch);
      save_decomposed_image_color(saving_path + chat_msg->data + "_Inc.png",
                                  cropped_Inc, colormap, Batch_Inc_map);
    } else {
      std::vector<cv::Vec3b> colormap = save_image_original_color(
          saving_path + chat_msg->data + "_Inc.png", image2save_Inc);
      save_decomposed_image_color(saving_path + chat_msg->data + "_Batch.png",
                                  Batch_segmentated, colormap, Batch_Inc_map);
    }
  }

  ////////////////////////
  // PUBLISHING METHODS
  ////////////////////////////
  void publish_Image() { image_pub_.publish(cv_ptr->toImageMsg()); }

  /////////////////////////
  //// UTILITY
  /////////////////////////

  cv::Mat clean_image(cv::Mat Occ_Image, cv::Mat &black_image) {
    // Occupancy Image to Free Space

    cv::Mat valid_image = Occ_Image < valid_space_threshold_;
    std::vector<std::vector<cv::Point>> test_contour;
    cv::findContours(valid_image, test_contour, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);

    cv::Rect first_rect = cv::boundingRect(test_contour[0]);
    for (int i = 1; i < test_contour.size(); i++) {
      first_rect |= cv::boundingRect(test_contour[i]);
    }
    cv::Mat reduced_Image;
    valid_image(first_rect).copyTo(reduced_Image);

    cv::Mat open_space = reduced_Image < free_space_threshold_;
    black_image = (reduced_Image > occupied_min_threshold_) &
                  (reduced_Image <= occupied_max_threshold_);
    cv::Mat Median_Image, out_image, temp_image;
    int filter_size = obstacle_filter_size_;

    cv::boxFilter(black_image, temp_image, -1,
                  cv::Size(filter_size, filter_size), cv::Point(-1, -1), false,
                  cv::BORDER_DEFAULT); // filter open_space
    black_image =
        temp_image > filter_size * filter_size / 2; // threshold in filtered
    cv::dilate(black_image, black_image, cv::Mat(), cv::Point(-1, -1),
               obstacle_dilation_iterations_, cv::BORDER_CONSTANT,
               cv::morphologyDefaultBorderValue()); // inflate obstacle

    filter_size = free_space_filter_size_;
    cv::boxFilter(open_space, temp_image, -1,
                  cv::Size(filter_size, filter_size), cv::Point(-1, -1), false,
                  cv::BORDER_DEFAULT); // filter open_space
    Median_Image =
        temp_image > filter_size * filter_size / 2; // threshold in filtered
    Median_Image = Median_Image | open_space;
    // cv::medianBlur(Median_Image, Median_Image, 3);
    cv::dilate(Median_Image, Median_Image, cv::Mat());

    out_image = Median_Image & ~black_image; // Open space without obstacles

    cv::Size image_size = Occ_Image.size();
    cv::Mat image_out(image_size, CV_8UC1);
    cv::Mat black_image_out(image_size, CV_8UC1);

    out_image.copyTo(image_out(first_rect));

    black_image.copyTo(black_image_out(first_rect));
    black_image = black_image_out;

    return image_out;
  }

  cv::Mat clean_image2(cv::Mat Occ_Image, cv::Mat &black_image) {
    // Occupancy Image to Free Space
    cv::Mat open_space = Occ_Image < free_space_threshold_;
    black_image = (Occ_Image > occupied_min_threshold_) &
                  (Occ_Image <= occupied_max_threshold_);
    cv::Mat Median_Image, out_image, temp_image;
    int filter_size = obstacle_filter_size_;

    cv::boxFilter(black_image, temp_image, -1,
                  cv::Size(filter_size, filter_size), cv::Point(-1, -1), false,
                  cv::BORDER_DEFAULT); // filter open_space
    black_image =
        temp_image > filter_size * filter_size / 2; // threshold in filtered
    cv::dilate(black_image, black_image, cv::Mat(), cv::Point(-1, -1),
               obstacle_dilation_iterations_,
               cv::BORDER_CONSTANT,
               cv::morphologyDefaultBorderValue()); // inflate obstacle

    filter_size = free_space_filter_size_;
    cv::boxFilter(open_space, temp_image, -1,
                  cv::Size(filter_size, filter_size), cv::Point(-1, -1), false,
                  cv::BORDER_DEFAULT); // filter open_space
    Median_Image =
        temp_image > filter_size * filter_size / 2; // threshold in filtered
    Median_Image = Median_Image | open_space;
    // cv::medianBlur(Median_Image, Median_Image, 3);
    cv::dilate(Median_Image, Median_Image, cv::Mat());

    out_image = Median_Image & ~black_image; // Open space without obstacles

    return out_image;
  }

  cv::Rect find_image_bounding_Rect(cv::Mat Occ_Image) {
    cv::Mat valid_image = Occ_Image < valid_space_threshold_;
    std::vector<std::vector<cv::Point>> test_contour;
    cv::findContours(valid_image, test_contour, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);
    if (test_contour.empty()) {
      return cv::Rect();
    }

    cv::Rect first_rect = cv::boundingRect(test_contour[0]);
    for (int i = 1; i < test_contour.size(); i++) {
      first_rect |= cv::boundingRect(test_contour[i]);
    }
    return first_rect;
  }

  /////////////////////////
  //// REGION TRACKING
  /////////////////////////

  void declare_region_tracking_parameters() {
    publish_missing_regions_ =
        this->declare_parameter<bool>("publish_missing_regions", false);
    debug_region_tracking_ =
        this->declare_parameter<bool>("debug_region_tracking", false);
    adapter_config_.adjacency_distance_cells = this->declare_parameter<double>(
        "region_tracking.adjacency_distance_cells",
        adapter_config_.adjacency_distance_cells);
    adapter_config_.adjacency_min_contact_m = this->declare_parameter<double>(
        "region_tracking.adjacency_min_contact_m",
        adapter_config_.adjacency_min_contact_m);

    inc_dude::TrackerConfig c;
    const std::string p = "region_tracking.";
    c.min_region_area_m2 =
        this->declare_parameter<double>(p + "min_region_area_m2", c.min_region_area_m2);
    c.overlap_sample_step_m = this->declare_parameter<double>(
        p + "overlap_sample_step_m", c.overlap_sample_step_m);
    c.gate_min_containment = this->declare_parameter<double>(
        p + "gate_min_containment", c.gate_min_containment);
    c.gate_centroid_distance_m = this->declare_parameter<double>(
        p + "gate_centroid_distance_m", c.gate_centroid_distance_m);
    c.gate_min_area_ratio = this->declare_parameter<double>(
        p + "gate_min_area_ratio", c.gate_min_area_ratio);
    c.weight_iou = this->declare_parameter<double>(p + "weight_iou", c.weight_iou);
    c.weight_containment = this->declare_parameter<double>(
        p + "weight_containment", c.weight_containment);
    c.weight_centroid =
        this->declare_parameter<double>(p + "weight_centroid", c.weight_centroid);
    c.weight_area =
        this->declare_parameter<double>(p + "weight_area", c.weight_area);
    c.centroid_distance_scale_m = this->declare_parameter<double>(
        p + "centroid_distance_scale_m", c.centroid_distance_scale_m);
    c.split_merge_min_fraction = this->declare_parameter<double>(
        p + "split_merge_min_fraction", c.split_merge_min_fraction);
    c.retire_min_coverage = this->declare_parameter<double>(
        p + "retire_min_coverage", c.retire_min_coverage);
    c.max_missed_updates = static_cast<int>(this->declare_parameter<int>(
        p + "max_missed_updates", c.max_missed_updates));
    c.max_area_loss_fraction = this->declare_parameter<double>(
        p + "max_area_loss_fraction", c.max_area_loss_fraction);
    c.max_region_overlap_fraction = this->declare_parameter<double>(
        p + "max_region_overlap_fraction", c.max_region_overlap_fraction);
    c.max_consecutive_rejections = static_cast<int>(this->declare_parameter<int>(
        p + "max_consecutive_rejections", c.max_consecutive_rejections));

    const std::string config_error = inc_dude::validateConfig(c);
    if (!config_error.empty()) {
      throw std::invalid_argument("Invalid region_tracking parameters: " +
                                  config_error);
    }
    if (!(adapter_config_.adjacency_distance_cells >= 0.0) ||
        !(adapter_config_.adjacency_min_contact_m >= 0.0)) {
      throw std::invalid_argument(
          "Invalid region_tracking adjacency parameters: must be >= 0");
    }
    tracker_ = std::make_unique<inc_dude::RegionTracker>(c);
  }

  // The incremental decomposer keeps its stable regions in pixel coordinates
  // and only compensates origin translations. Maps in another frame are
  // rejected; a resolution or rotation change restarts the decomposer (the
  // tracker keeps the canonical ids, matching in metric map coordinates).
  bool check_grid_consistency(const std::string &frame_id,
                              const inc_dude::GridGeometry &grid) {
    if (!have_grid_) {
      have_grid_ = true;
      grid_frame_id_ = frame_id;
      grid_ = grid;
      return true;
    }
    if (frame_id != grid_frame_id_) {
      RCLCPP_WARN(this->get_logger(),
                  "Ignoring map in frame '%s', expected '%s'", frame_id.c_str(),
                  grid_frame_id_.c_str());
      return false;
    }
    const bool resolution_changed =
        std::abs(grid.resolution - grid_.resolution) > 1e-6;
    const bool yaw_changed =
        std::abs(std::remainder(grid.origin_yaw - grid_.origin_yaw,
                                2.0 * M_PI)) > 1e-6;
    if (resolution_changed || yaw_changed) {
      RCLCPP_WARN(this->get_logger(),
                  "Map %s changed; restarting the incremental decomposition",
                  resolution_changed ? "resolution" : "orientation");
      inc_decomp = Incremental_Decomposer();
      Stable = Stable_graph();
    }
    grid_ = grid;
    return true;
  }

  void track_and_publish_regions(const std_msgs::msg::Header &header,
                                 const inc_dude::GridGeometry &grid) {
    inc_dude::RegionUpdate update;
    update.frame_id = header.frame_id;
    update.stamp_ns = rclcpp::Time(header.stamp).nanoseconds();
    update.resolution = grid.resolution;
    update.regions = inc_dude::regionsFromContours(Stable.Region_contour, grid,
                                                   adapter_config_);

    const inc_dude::TrackerUpdateResult result = tracker_->update(update);
    if (!result.accepted) {
      RCLCPP_WARN(this->get_logger(),
                  "Region update rejected (%zu raw regions), keeping last "
                  "valid tracking state: %s",
                  Stable.Region_contour.size(),
                  result.rejection_reason.c_str());
      return;
    }
    if (result.forced) {
      RCLCPP_WARN(this->get_logger(),
                  "Region update accepted after %d consecutive rejections: %s",
                  tracker_->config().max_consecutive_rejections,
                  result.rejection_reason.c_str());
    }
    if (debug_region_tracking_) {
      RCLCPP_INFO(this->get_logger(), "[region tracking] %s",
                  inc_dude::formatUpdateReport(result, true).c_str());
    }

    const auto regions_msg = inc_dude::toMsg(
        tracker_->publishableTracks(publish_missing_regions_), result.events,
        header, result.update_index);
    regions_pub_->publish(regions_msg);
    if (region_markers_pub_) {
      region_markers_pub_->publish(inc_dude::toMarkers(regions_msg));
    }
  }

  /////////////////
  void save_images_color(cv::Mat DuDe_segmentation) {
    std::string full_path_decomposed =
        segmentation_output_directory_ + "/map_decomposed.png";
    double min, max;

    std::vector<cv::Vec3b> color_vector;
    cv::Vec3b black(0, 0, 0);
    color_vector.push_back(black);

    cv::minMaxLoc(DuDe_segmentation, &min, &max);

    for (int i = 0; i <= max; i++) {
      cv::Vec3b color(rand() % 255, rand() % 255, rand() % 255);
      color_vector.push_back(color);
    }
    cv::Mat DuDe_segmentation_float =
        cv::Mat::zeros(DuDe_segmentation.size(), CV_8UC3);
    for (int i = 0; i < DuDe_segmentation.rows; i++) {
      for (int j = 0; j < DuDe_segmentation.cols; j++) {
        int color_index = DuDe_segmentation.at<uchar>(i, j);
        DuDe_segmentation_float.at<cv::Vec3b>(i, j) = color_vector[color_index];
      }
    }
    cv::imwrite(full_path_decomposed, DuDe_segmentation_float);
  }

  /////////////////////////////////////
  //// Evaluation
  //////////////////////////////////
  typedef std::map<std::vector<int>, std::vector<cv::Point>> match2points;
  typedef std::map<int, std::vector<cv::Point>> tag2points;
  typedef std::map<int, tag2points> tag2tagMapper;

  /////////////////
  void save_decomposed_image_color(std::string path, cv::Mat image_in,
                                   std::vector<cv::Vec3b> colormap,
                                   std::map<int, int> original_map) {
    double min, max;
    std::vector<cv::Vec3b> color_vector;
    cv::Vec3b black(208, 208, 208);
    color_vector.push_back(black);

    std::map<int, int>::iterator map_iter;

    cv::minMaxLoc(image_in, &min, &max);
    color_vector.resize(max);

    for (int i = 1; i <= max; i++) {
      map_iter = original_map.find(i);
      if (map_iter != original_map.end()) {
        int index_in_original = map_iter->second;
        color_vector[i] = colormap[index_in_original];
      } else {
        cv::Vec3b color(rand() % 255, rand() % 255, rand() % 255);
        color_vector[i] = color;
      }
    }
    /////
    cv::Mat image_float = cv::Mat::zeros(image_in.size(), CV_8UC3);
    for (int i = 0; i < image_in.rows; i++) {
      for (int j = 0; j < image_in.cols; j++) {
        int color_index = image_in.at<uchar>(i, j);
        image_float.at<cv::Vec3b>(i, j) = color_vector[color_index];
      }
    }
    cv::imwrite(path, image_float);
  }

  /////////////////
  std::vector<cv::Vec3b> save_image_original_color(std::string path,
                                                   cv::Mat image_in) {

    double min, max;
    std::vector<cv::Vec3b> color_vector;
    cv::Vec3b black(208, 208, 208);
    color_vector.push_back(black);

    cv::minMaxLoc(image_in, &min, &max);

    for (int i = 0; i <= max; i++) {
      cv::Vec3b color(rand() % 255, rand() % 255, rand() % 255);
      color_vector.push_back(color);
    }
    cv::Mat image_float = cv::Mat::zeros(image_in.size(), CV_8UC3);
    for (int i = 0; i < image_in.rows; i++) {
      for (int j = 0; j < image_in.cols; j++) {
        int color_index = image_in.at<uchar>(i, j);
        image_float.at<cv::Vec3b>(i, j) = color_vector[color_index];
      }
    }
    cv::imwrite(path, image_float);

    return color_vector;
  }

  ////////////////////
  cv::Mat simple_segment(cv::Mat image_in) {
    Incremental_Decomposer inc_decomp;
    Stable_graph Stable;
    cv::Point2f origin(0, 0);
    float resolution = offline_image_resolution_;

    cv::Mat pre_decompose = image_in.clone();
    cv::Mat pre_decompose_BW = pre_decompose > offline_free_threshold_;
    //			cv::Mat pre_decompose_BW = clean_image(pre_decompose >
    //250);

    Stable = inc_decomp.decompose_image(
        pre_decompose_BW, Decomp_threshold_ / resolution, origin, resolution);

    //			cv::Mat Segmentation = Stable.draw_stable_contour();

    cv::Mat Drawing = cv::Mat::zeros(image_in.size(), CV_8UC1);
    for (int i = 0; i < Stable.Region_contour.size(); i++) {
      drawContours(Drawing, Stable.Region_contour, i, i + 1, -1, 8);
    }
    std::cout << "Decomposition size: " << Stable.Region_contour.size()
              << std::endl;

    return Drawing;
  }

  /////////////////////
  std::map<int, int> compare_images(cv::Mat GT_segmentation_in,
                                    cv::Mat DuDe_segmentation_in) {

    std::map<int, int> segmented2GT_tags;

    cv::Mat GT_segmentation =
        cv::Mat::zeros(GT_segmentation_in.size(), CV_8UC1);
    cv::Mat DuDe_segmentation =
        cv::Mat::zeros(GT_segmentation_in.size(), CV_8UC1);

    GT_segmentation_in.convertTo(GT_segmentation, CV_8UC1);
    DuDe_segmentation_in.convertTo(DuDe_segmentation, CV_8UC1);
    tag2tagMapper gt_tag2mapper, DuDe_tag2mapper;

    match2points links2points;
    tag2points GT_points, DuDe_points;

    for (int x = 0; x < GT_segmentation.size().width; x++) {
      for (int y = 0; y < GT_segmentation.size().height; y++) {
        cv::Point current_pixel(x, y);
        std::vector<int> match;

        int tag_GT = GT_segmentation.at<uchar>(current_pixel);
        int tag_DuDe = DuDe_segmentation.at<uchar>(current_pixel);

        if (tag_DuDe > 0 && tag_GT > 0) {
          gt_tag2mapper[tag_GT][tag_DuDe].push_back(current_pixel);
          DuDe_tag2mapper[tag_DuDe][tag_GT].push_back(current_pixel);
          match.push_back(tag_DuDe);
          match.push_back(tag_GT);
          links2points[match].push_back(current_pixel);
          GT_points[tag_GT].push_back(current_pixel);
          DuDe_points[tag_DuDe].push_back(current_pixel);
        }
      }
    }
    /*
    for(tag2points::iterator it = GT_points.begin(); it != GT_points.end();
    it++) std::cout << "GT "<<it->first<<", with size " <<it->second.size() << "
    size2  " << GT_points[it->first].size()  <<endl; for(tag2points::iterator it
    = DuDe_points.begin(); it != DuDe_points.end(); it++) std::cout << "DuDe
    "<<it->first<<", with size" <<it->second.size() << endl;
    //*/

    std::map<std::vector<int>, float> link2relation;
    std::map<int, int> DuDe_Union_Match;
    int current_DuDe_Tag = 0;
    int current_GT_max = -1;
    int current_DuDe_evaluated = -1;
    float current_GT_max_relation = -1;

    for (match2points::iterator it = links2points.begin();
         it != links2points.end(); it++) {
      std::vector<cv::Point> points_in_match = it->second;

      float A = GT_points[it->first[1]].size();
      float B = DuDe_points[it->first[0]].size();
      float AandB = points_in_match.size();
      float relation = AandB / (A + B - AandB);

      link2relation[it->first] = relation;
      //				std::cout <<
      //"["<<it->first[0]<<","<<it->first[1]<<"] has "<< relation << endl;
      //				std::cout << "  AandB "<<AandB<<", A "<<
      //A <<", B "<< B << endl;

      if (current_DuDe_evaluated != it->first[0]) { // update maximum
        DuDe_Union_Match[current_DuDe_evaluated] = current_GT_max;
        //					std::cout << "    Maximum is
        //["<< current_DuDe_evaluated <<","<< current_GT_max <<"] with relation
        //"<< current_GT_max_relation << endl;
        current_DuDe_evaluated = it->first[0];
        current_GT_max = it->first[1];
        current_GT_max_relation = relation;
      } else if (relation > current_GT_max_relation) {
        current_GT_max = it->first[1];
        current_GT_max_relation = relation;
      }
    }
    DuDe_Union_Match[current_DuDe_evaluated] = current_GT_max;
    //			std::cout << "    Maximum is ["<< current_DuDe_evaluated
    //<<","<< current_GT_max <<"] with relation "<< current_GT_max_relation <<
    //endl;
    /*


    for( tag2tagMapper::iterator it = gt_tag2mapper.begin(); it!=
    gt_tag2mapper.end(); it++ ){ tag2points inside = it->second; int
    max_intersection=0, total_points=0; int gt_tag_max = -1; for(
    tag2points::iterator it2 = inside.begin(); it2!= inside.end(); it2++ ){
                    total_points += it2->second.size();
                    if (it2->second.size() > max_intersection){
                            max_intersection = it2->second.size();
                            gt_tag_max = it2->first;
                    }
            }
            segmented2GT_tags[gt_tag_max] = it->first;
    }


    for( tag2tagMapper::iterator it = DuDe_tag2mapper.begin(); it!=
    DuDe_tag2mapper.end(); it++ ){ tag2points inside = it->second; int
    max_intersection=0, total_points=0; for( tag2points::iterator it2 =
    inside.begin(); it2!= inside.end(); it2++ ){ total_points +=
    it2->second.size(); if (it2->second.size() > max_intersection)
    max_intersection = it2->second.size();
            }
    }


    //*/
    //			return(segmented2GT_tags);
    return DuDe_Union_Match;
  }
};

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);

  std::string mapname = "map";

  float decomp_th = 3;
  if (argc == 2) {
    decomp_th = atof(argv[1]);
  }

  auto mg = std::make_shared<ROS_handler>(mapname, decomp_th);
  rclcpp::spin(mg);
  rclcpp::shutdown();

  return 0;
}
