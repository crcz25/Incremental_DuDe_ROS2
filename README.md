# Incremental Contour Based Segmentation

Implementation of Incremental Contour-Based Topological Segmentation in structured or unstructured environments.
The package transforms an occupancy grid into a set of polygons and uses [Dual Space Decomposition](http://masc.cs.gmu.edu/wiki/Dude2D) to segment them. The incremental version only decomposes the differences between maps.

## Prerequisites

This is a native ROS 2 package. It has been ported and tested with [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/).

Clone the repository with its bundled third-party dependencies:

```bash
git submodule update --init --recursive
```

The DuDe dependencies are vendored under `Third_Party`:

- `Third_Party/dude_final`
- `Third_Party/freeglut`
- `Third_Party/cgal`
- `Third_Party/mpfr`

CGAL, freeglut, and MPFR are no longer discovered through `cmake_modules` or installed as system CGAL/MPFR packages. CMake builds/uses them from `Third_Party` directly. The old `cmake_modules/FindMPFR.cmake` lookup is not used.

MPFR still requires GMP and autotools to build from source:

```bash
sudo apt-get install libgmp-dev autoconf automake libtool
```

Install the ROS 2 dependencies with rosdep after sourcing ROS 2:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths . --ignore-src -r -y
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Configuration

Runtime parameters are defined in `config/inc_dude_params.yaml`. The file is installed with the package and loaded by the ROS 2 launch files by default.

The `incremental_decomposer` parameters are:

- `decomp_threshold`: concavity threshold in meters.
- `map_topic`: input occupancy grid topic.
- `save_trigger_topic`: string topic that triggers saving comparison images.
- `tagged_image_topic`: output tagged decomposition image topic.
- `publish_period_ms`: tagged image republish period.
- `maps_path`: base map directory; empty uses the package `maps` directory.
- `segmentation_output_directory`: saved segmentation output directory; empty uses `maps_path/Topological_Segmentation`.
- `valid_space_threshold`, `free_space_threshold`, `occupied_min_threshold`, `occupied_max_threshold`: occupancy-grid cleanup thresholds.
- `obstacle_filter_size`, `free_space_filter_size`, `obstacle_dilation_iterations`: OpenCV cleanup filter and dilation settings.
- `offline_image_resolution`, `offline_free_threshold`: settings used by the save-trigger batch comparison path.

The `evaluation` parameters are:

- `decomp_threshold`: concavity threshold in meters.
- `cmd_vel_topic`: twist topic used to cycle/process evaluation files.
- `ground_truth_segmentation_topic`, `dude_segmentation_topic`, `inc_dude_segmentation_topic`: evaluation image output topics.
- `publish_period_ms`: evaluation image republish period.
- `dataset_path`: evaluation image directory; empty uses the packaged dataset.
- `ground_truth_suffix`, `furniture_suffix`, `no_furniture_suffix`: dataset filename suffixes.
- `image_resolution`, `binary_free_threshold`: evaluation image decomposition settings.
- `scan_step_pixels`, `scan_radius_pixels`, `min_scan_free_pixels`: synthetic incremental scan settings.
- `initial_file_index`: initial sorted dataset file index.

To use a different parameter file:

```bash
ros2 launch inc_dude inc_dude.launch.py config_file:=/path/to/inc_dude_params.yaml
ros2 launch inc_dude evaluation.launch.py config_file:=/path/to/inc_dude_params.yaml
```

## Description

### OpenCV Wrapper

Transforms a `cv::Mat` image into contours and decomposes it using [DuDe](http://masc.cs.gmu.edu/wiki/Dude2D). This part does not require ROS.

### Incremental Decomposition

Transforms an occupancy grid into an OpenCV binary image, compares it to the previous decomposition, and decomposes the resulting difference image with the OpenCV wrapper.

### ROS 2 Nodes

#### `inc_dude`

Subscribes to `/map` and publishes the decomposition image on `/tagged_image`. The `/chatter` topic is used to save the current decomposition image.

```bash
source install/setup.bash
ros2 run inc_dude inc_dude --ros-args -p decomp_threshold:=3.0
```

The same node can be launched with:

```bash
ros2 launch inc_dude inc_dude.launch.py
```

#### `evaluation`

Processes the bundled dataset and publishes images on `/ground_truth_segmentation`, `/DuDe_segmentation`, and `/inc_dude_segmentation`. The `/cmd_vel` topic is used to cycle and process images.

```bash
source install/setup.bash
ros2 run inc_dude evaluation --ros-args -p decomp_threshold:=3.0
```

## Related Papers

- Fermin L., Neira J. and Castellanos, J.A. Incremental Contour-Based Topological Segmentation for Robot Exploration. *2017 IEEE International Conference on Robotics and Automation, ICRA 2017*. (*submitted*)
