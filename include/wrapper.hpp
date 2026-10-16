#pragma once  // M-26
#include <memory>

// openCV
#include <opencv2/imgproc/imgproc.hpp>

// DuDe
#include "SE2d.h"
#include "SE2d_skeleton.h"
#include "draw_decoration.h"
#include "dude.h"
#include "dude_use.h"

#include "Quaternion.h"
#include "diagonal2.h"
#include "dude_param.h"
#include "dude_use.h"
#include "gettime.h"
#include "holediag.h"
#include "polygon.h"
#include "polyline.h"

class DuDe_OpenCV_wrapper {

public:
  string g_filename;          // filename
  string g_concavity_measure; // concavity_measure name
  double g_alpha, g_beta;     // for selecting cutting direction

  double g_tau; // tolerance in percentage 0-1
  float Area_threshold;
  float g_tau_pixel; // tolerance in pixels
  bool tolerance_in_pixels;

  // DuDe Related
  c_dude dude;
  Draw_Decoration draw_decoration;
  vector<c_polygon *> finalPolygonPieces; // the result polygons, they can be
                                          // exported as individual files
  // H-03: what the wrapper allocates for DuDe, released with it. The copy
  // of the input that dude.build() and iterativeDecompose() work on (the
  // skeleton in draw_decoration.se points into it; the destructor frees its
  // vertices with destroy()), and the skeleton extractor
  // (ES_Factory::create_ES("PA") is new ES_PA, whose base class has no
  // virtual destructor).
  std::unique_ptr<c_polygon> decomposed_polygon;
  std::unique_ptr<ES_PA> skeleton_extractor;
  //////

  std::vector<cv::Point> Parent_contour;

  //	Nodes
  std::vector<std::vector<cv::Point>> Decomposed_contours;
  std::vector<cv::Point> contours_centroid;
  std::vector<std::set<int>> contours_connections;

  // Edges
  std::vector<cv::Point> diagonal_centroid;
  std::vector<std::set<int>> diagonal_connections;

  DuDe_OpenCV_wrapper();

  ~DuDe_OpenCV_wrapper();

  ///////////////////////////////////
  void set_Tau(float decomp_th) {
    g_tau = decomp_th;
    tolerance_in_pixels = false;
  }
  ///////////////////////////////////
  void set_pixel_Tau(float decomp_th) {
    g_tau_pixel = decomp_th;
    tolerance_in_pixels = true;
  }
  ///////////////////////////////////
  void insert_contour_to_poly(std::vector<cv::Point> contour_in,
                              c_ply &polygon);
  ////////////////////////////////////////
  cv::Rect Decomposer(cv::Mat Occ_Image);
  //////////////////////////////////////////////////////////////
  void Dual_decompose(c_polygon &polygons);
  //////////////////////
  void prepare_skeleton();
  //////////////////////////////
  void export_all_svg_files();
  /////////////////////////////////////
  void estimate_COM_R_Box();
  /////////////////////////////
  void extract_contour_from_polygon();
  ///////////////////////////////////////
  void extract_graph();
  ////////////////////////////
  void print_graph();
  ////////////////////////////
  void measure_performance();
  /////////////////////////////////////////
};
