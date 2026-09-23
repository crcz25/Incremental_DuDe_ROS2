cmake_minimum_required(VERSION 3.8)
project(inc_dude)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
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

add_library(cgal_headers INTERFACE)
target_include_directories(cgal_headers INTERFACE
  ${PROJECT_SOURCE_DIR}/Third_Party/cgal/include
)

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

include_directories(
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/mathtool
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src/skeleton
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src/draw
  ${PROJECT_SOURCE_DIR}/include
)

file(GLOB DuDe_SOURCES
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src/*.cpp
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src/skeleton/*.cpp
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src/graph/*.cpp
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/src/draw/*.cpp
)

set(DUDE_COMMON_SOURCES
  include/wrapper.cpp
  include/inc_decomp.cpp
  ${DuDe_SOURCES}
)

set(ROS_DEPENDENCIES
  rclcpp
  ament_index_cpp
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  cv_bridge
  image_transport
)

set(FIST_LIBRARIES
  ${PROJECT_SOURCE_DIR}/Third_Party/dude_final/FIST/libFIST.linux64.a
)

function(add_inc_dude_executable target source)
  add_executable(${target} ${source} ${DUDE_COMMON_SOURCES})
  ament_target_dependencies(${target} ${ROS_DEPENDENCIES})
  add_dependencies(${target} third_party_dependencies)
  target_link_options(${target} PRIVATE -no-pie)
  target_link_libraries(${target}
    ${FIST_LIBRARIES}
    ${OpenCV_LIBRARIES}
    cgal_headers
    mpfr_static
    ${GMP_LIBRARY}
  )
endfunction()

add_inc_dude_executable(inc_dude_node src/inc_dude.cpp)
set_target_properties(inc_dude_node PROPERTIES OUTPUT_NAME inc_dude)
target_link_libraries(inc_dude_node inc_dude_region_msgs)
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
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
