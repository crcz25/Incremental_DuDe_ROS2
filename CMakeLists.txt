cmake_minimum_required(VERSION 3.8)
project(inc_dude)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# The interface target must be ${PROJECT_NAME}: typesupport libraries are
# looked up at runtime by package name. The node target is therefore called
# inc_dude_node and installed under the executable name inc_dude.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Region2D.msg"
  "msg/RegionEvent.msg"
  "msg/Region2DArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)
rosidl_get_typesupport_target(inc_dude_cpp_typesupport
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Region tracking core: ROS-independent, unit tested directly.
add_library(inc_dude_tracking STATIC
  src/tracking/region_geometry.cpp
  src/tracking/decomposition_adapter.cpp
  src/tracking/hungarian.cpp
  src/tracking/region_tracker.cpp
)
target_include_directories(inc_dude_tracking PUBLIC
  ${PROJECT_SOURCE_DIR}/include
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(inc_dude_tracking PUBLIC opencv_core)
set_target_properties(inc_dude_tracking PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Paths of the images the save trigger writes.
add_library(inc_dude_save_path STATIC src/save_path.cpp)
target_include_directories(inc_dude_save_path PUBLIC ${PROJECT_SOURCE_DIR}/include)
set_target_properties(inc_dude_save_path PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ROS message conversion for tracked regions.
add_library(inc_dude_region_msgs STATIC
  src/tracking/region_msg_conversion.cpp
)
target_link_libraries(inc_dude_region_msgs PUBLIC
  inc_dude_tracking
  "${inc_dude_cpp_typesupport}"
)
ament_target_dependencies(inc_dude_region_msgs PUBLIC
  std_msgs geometry_msgs nav_msgs visualization_msgs)

set(FREEGLUT_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(FREEGLUT_BUILD_STATIC_LIBS ON CACHE BOOL "" FORCE)
set(FREEGLUT_BUILD_DEMOS OFF CACHE BOOL "" FORCE)
add_subdirectory(Third_Party/freeglut EXCLUDE_FROM_ALL)

# CGAL is header-only and comes from the system package libcgal-dev (5.4 on
# jammy; docker/Dockerfile, the CI workflow). Third_Party/cgal is the upstream
# development tree, which has no include/ folder, so it is not used. The DuDe
# golden files were made with CGAL 5.4.

find_path(GMP_INCLUDE_DIR gmp.h REQUIRED)
find_library(GMP_LIBRARY gmp REQUIRED)
get_filename_component(GMP_LIBRARY_DIR ${GMP_LIBRARY} DIRECTORY)

set(MPFR_SOURCE_DIR ${PROJECT_SOURCE_DIR}/Third_Party/mpfr)
set(MPFR_BUILD_ROOT ${CMAKE_CURRENT_BINARY_DIR}/third_party/mpfr)
set(MPFR_BUILD_SOURCE_DIR ${MPFR_BUILD_ROOT}/src)
set(MPFR_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/third_party/mpfr-install)
set(MPFR_LIBRARY ${MPFR_INSTALL_DIR}/lib/libmpfr.a)
set(MPFR_INCLUDE_DIR ${MPFR_INSTALL_DIR}/include)
file(MAKE_DIRECTORY ${MPFR_INCLUDE_DIR})

add_custom_command(
  OUTPUT ${MPFR_LIBRARY}
  COMMAND ${CMAKE_COMMAND} -E rm -rf ${MPFR_BUILD_SOURCE_DIR} ${MPFR_INSTALL_DIR}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${MPFR_BUILD_ROOT}
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${MPFR_SOURCE_DIR} ${MPFR_BUILD_SOURCE_DIR}
  COMMAND ${CMAKE_COMMAND} -E chdir ${MPFR_BUILD_SOURCE_DIR} ./autogen.sh
  COMMAND ${CMAKE_COMMAND} -E chdir ${MPFR_BUILD_SOURCE_DIR}
          ${CMAKE_COMMAND} -E env MAKEINFO=true
          CPPFLAGS=-I${GMP_INCLUDE_DIR}
          LDFLAGS=-L${GMP_LIBRARY_DIR}
          ./configure --prefix=${MPFR_INSTALL_DIR}
          --disable-shared --enable-static
  COMMAND ${CMAKE_COMMAND} -E chdir ${MPFR_BUILD_SOURCE_DIR}
          ${CMAKE_COMMAND} -E env MAKEINFO=true ${CMAKE_MAKE_PROGRAM}
  COMMAND ${CMAKE_COMMAND} -E chdir ${MPFR_BUILD_SOURCE_DIR}
          ${CMAKE_COMMAND} -E env MAKEINFO=true ${CMAKE_MAKE_PROGRAM} install
  VERBATIM
)

add_custom_target(mpfr_external DEPENDS ${MPFR_LIBRARY})

add_library(mpfr_static STATIC IMPORTED GLOBAL)
set_target_properties(mpfr_static PROPERTIES
  IMPORTED_LOCATION ${MPFR_LIBRARY}
  INTERFACE_INCLUDE_DIRECTORIES ${MPFR_INCLUDE_DIR}
)
add_dependencies(mpfr_static mpfr_external)

add_custom_target(third_party_dependencies
  DEPENDS freeglut_static mpfr_external
)

set(DUDE_INCLUDE_DIRS
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/mathtool
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src/skeleton
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src/draw
)

file(GLOB DuDe_SOURCES
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src/*.cpp
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src/skeleton/*.cpp
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src/graph/*.cpp
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src/draw/*.cpp
)

set(FIST_LIBRARIES
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/FIST/libFIST.linux64.a
)

# M-32: DuDe and its wrapper are compiled once, into one static library that
# the node, the evaluation node and their tests link. DuDe is vendored code:
# it is compiled without warnings (-w), and its headers are SYSTEM headers for
# first-party code. Its sources are not changed (patches/dude/).
set_source_files_properties(${DuDe_SOURCES} PROPERTIES COMPILE_OPTIONS "-w")
add_library(inc_dude_dude STATIC
  src/wrapper.cpp
  src/inc_decomp.cpp
  ${DuDe_SOURCES}
)
target_include_directories(inc_dude_dude SYSTEM PUBLIC ${DUDE_INCLUDE_DIRS})
target_include_directories(inc_dude_dude PUBLIC ${PROJECT_SOURCE_DIR}/include)
add_dependencies(inc_dude_dude third_party_dependencies)
target_link_libraries(inc_dude_dude PUBLIC
  ${FIST_LIBRARIES}
  ${OpenCV_LIBRARIES}
  mpfr_static
  ${GMP_LIBRARY}
)

set(ROS_DEPENDENCIES
  rclcpp
  ament_index_cpp
  std_msgs
  std_srvs
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  image_transport
)

function(add_inc_dude_executable target source)
  add_executable(${target} ${source})
  ament_target_dependencies(${target} ${ROS_DEPENDENCIES})
  target_link_options(${target} PRIVATE -no-pie)
  target_link_libraries(${target} inc_dude_dude)
endfunction()

add_inc_dude_executable(inc_dude_node src/inc_dude.cpp)
set_target_properties(inc_dude_node PROPERTIES OUTPUT_NAME inc_dude)
target_link_libraries(inc_dude_node inc_dude_region_msgs inc_dude_save_path)
add_inc_dude_executable(evaluation src/evaluation.cpp)

install(TARGETS
  inc_dude_node
  evaluation
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_region_tracking
    test/test_region_geometry.cpp
    test/test_decomposition_adapter.cpp
    test/test_region_tracker.cpp
  )
  target_link_libraries(test_region_tracking inc_dude_tracking)

  # Names and paths of the images the save trigger writes (H-01).
  ament_add_gtest(test_timing_stats test/test_timing_stats.cpp)
  target_include_directories(test_timing_stats PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  ament_add_gtest(test_save_path test/test_save_path.cpp)
  target_link_libraries(test_save_path inc_dude_save_path)

  # ROS message conversion of tracked regions.
  ament_add_gtest(test_region_msg_conversion test/test_region_msg_conversion.cpp)
  target_link_libraries(test_region_msg_conversion inc_dude_region_msgs)

  # The node's save path: test/test_inc_dude_node.cpp compiles src/inc_dude.cpp
  # with main() renamed, so it links like inc_dude_node. Creates a node, so
  # it gets its own ROS_DOMAIN_ID.
  find_package(ament_cmake_ros REQUIRED)
  ament_add_ros_isolated_gtest(test_inc_dude_node test/test_inc_dude_node.cpp
    TIMEOUT 300 ENV "GTEST_FILTER=-DudeReference.*")
  # DudeReference.* (the DuDe reference output) runs in a process of its own:
  # DuDe's results depend on the heap layout that earlier decompositions in
  # the same process leave.
  ament_add_test(test_dude_reference
    COMMAND "$<TARGET_FILE:test_inc_dude_node>"
      "--gtest_output=xml:${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/test_dude_reference.gtest.xml"
    RESULT_FILE "${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/test_dude_reference.gtest.xml"
    ENV "GTEST_FILTER=DudeReference.*"
    TIMEOUT 300)
  ament_target_dependencies(test_inc_dude_node ${ROS_DEPENDENCIES})
  # M-24: the shipped parameter file loads.
  target_compile_definitions(test_inc_dude_node PRIVATE
    INC_DUDE_PARAMS_FILE="${CMAKE_CURRENT_SOURCE_DIR}/config/inc_dude_params.yaml")
  target_link_options(test_inc_dude_node PRIVATE -no-pie)
  target_link_libraries(test_inc_dude_node
    inc_dude_region_msgs
    inc_dude_save_path
    inc_dude_dude
  )

  # The evaluation node: test/test_evaluation_node.cpp compiles
  # src/evaluation.cpp with main() renamed, so it links like evaluation.
  ament_add_ros_isolated_gtest(test_evaluation_node test/test_evaluation_node.cpp
    TIMEOUT 300)
  ament_target_dependencies(test_evaluation_node ${ROS_DEPENDENCIES})
  target_link_options(test_evaluation_node PRIVATE -no-pie)
  target_link_libraries(test_evaluation_node inc_dude_dude)

  # Static analysis, opt-in with -DENABLE_LINT=ON (tools/ci/run_ci.sh): the
  # default colcon test runs no lint test.
  option(ENABLE_LINT "Add the cppcheck and clang-tidy tests" OFF)
  if(ENABLE_LINT)
    find_package(ament_cmake_cppcheck REQUIRED)
    ament_cppcheck(include src test
      LANGUAGE c++ INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include)
    # ament_cppcheck otherwise skips cppcheck 1.88 and 2.x as too slow.
    set_tests_properties(cppcheck PROPERTIES
      ENVIRONMENT "AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1")
    find_package(ament_cmake_clang_tidy QUIET)
    if(ament_cmake_clang_tidy_FOUND)
      # The workspace .clang-tidy, in this directory or the nearest parent.
      set(_clang_tidy_dir "${CMAKE_CURRENT_SOURCE_DIR}")
      while(NOT EXISTS "${_clang_tidy_dir}/.clang-tidy" AND
            NOT _clang_tidy_dir STREQUAL "/")
        get_filename_component(_clang_tidy_dir "${_clang_tidy_dir}" DIRECTORY)
      endwhile()
      # ament_clang_tidy lints every entry of compile_commands.json and has
      # no exclude option: give it a copy without the vendored Third_Party/
      # sources, the rosidl-generated sources in the build tree and, as for
      # the other packages, test/ (ament skips <package>/test/, and this
      # package's directory is incremental_dude).
      set(_clang_tidy_db_dir "${CMAKE_CURRENT_BINARY_DIR}/clang_tidy/${PROJECT_NAME}")
      add_custom_target(clang_tidy_compile_commands ALL
        COMMAND ${CMAKE_COMMAND} -E make_directory "${_clang_tidy_db_dir}"
        COMMAND ${CMAKE_COMMAND}
          -DINPUT=${CMAKE_BINARY_DIR}/compile_commands.json
          -DOUTPUT=${_clang_tidy_db_dir}/compile_commands.json
          "-DEXCLUDE=/Third_Party/|/test/|^${CMAKE_BINARY_DIR}/"
          -P ${CMAKE_CURRENT_SOURCE_DIR}/cmake/filter_compile_commands.cmake
        VERBATIM)
      # ament_clang_tidy passes its own --header-filter (include/<package>/),
      # which overrides HeaderFilterRegex, and runs one job under a 300 s
      # timeout: report this package's own headers, on every core.
      cmake_host_system_information(RESULT _clang_tidy_jobs
        QUERY NUMBER_OF_LOGICAL_CORES)
      ament_clang_tidy("${_clang_tidy_db_dir}"
        --jobs ${_clang_tidy_jobs}
        --header-filter "${CMAKE_CURRENT_SOURCE_DIR}/(include|src|test)/.*"
        CONFIG_FILE "${_clang_tidy_dir}/.clang-tidy"
        TIMEOUT 1800)
    else()
      message(WARNING "ament_cmake_clang_tidy not found: no clang_tidy test")
    endif()
  endif()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
