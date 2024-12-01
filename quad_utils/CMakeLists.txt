cmake_minimum_required(VERSION 3.0.2)
project(quad_utils)

## Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

## Set default cmake build type to release
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospack
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  quad_msgs
  grid_map_core
  grid_map_ros
  grid_map_pcl
  gazebo_msgs
  gazebo_ros
)

find_package(PythonLibs 2.7 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
find_package(gazebo REQUIRED)

# We need to add the project source path to the CMake module path so that
# the FindRBDL.cmake script can be found.
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

# Search for the RBDL include directory and library
FIND_PACKAGE(RBDL COMPONENTS URDFREADER REQUIRED)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include ${PYTHON_INCLUDE_DIRS}
  LIBRARIES quad_utils
  CATKIN_DEPENDS roscpp std_msgs nav_msgs sensor_msgs geometry_msgs quad_msgs grid_map_core grid_map_ros grid_map_pcl
  DEPENDS gazebo Boost Eigen3
)

## Set the C++ standard to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
include_directories(${RBDL_INCLUDE_DIR})

# link_directories(${GAZEBO_LIBRARY_DIRS})

## Declare a C++ library
add_library(quad_utils
  src/rviz_interface.cpp
  src/fast_terrain_map.cpp
  src/terrain_map_publisher.cpp
  src/trajectory_publisher.cpp
  src/mesh_to_grid_map_converter.cpp
  src/math_utils.cpp
  src/ros_utils.cpp
  src/quad_kd.cpp
  src/remote_heartbeat.cpp
  src/apply_force.cpp
  src/bounding_boxes.cpp
  src/yaml_loader.cpp
  src/collision_checker.cpp
  src/obstacle_publisher.cpp
  src/global_variables.cpp
  src/obstacle_sdf_name_publisher.cpp
  src/bbox_publish_node.cpp
)

# add_library(${PROJECT_NAME} SHARED src/bounding_boxes_plugin.cpp)

## Link libraries to quad_utils
target_link_libraries(quad_utils
  ${catkin_LIBRARIES}
  ${RBDL_LIBRARY}
  ${RBDL_URDFREADER_LIBRARY}
  yaml-cpp
)

# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

## Declare a C++ executable
add_executable(rviz_interface_node src/rviz_interface_node.cpp)
add_executable(terrain_map_publisher_node src/terrain_map_publisher_node.cpp)
add_executable(trajectory_publisher_node src/trajectory_publisher_node.cpp)
add_executable(mesh_to_grid_map_node src/mesh_to_grid_map_node.cpp)
add_executable(remote_heartbeat_node src/remote_heartbeat_node.cpp)
add_executable(apply_force_node src/apply_force_node.cpp)
add_executable(test_bounding_boxes_node src/test_bounding_boxes.cpp)
add_executable(test_collision_checker_node src/test_collision_checker.cpp)
add_executable(obstacle_publisher_node src/obstacle_publisher_node.cpp)
add_executable(obstacle_sdf_name_publisher_node src/obstacle_sdf_name_publisher_node.cpp)
add_executable(bboxes_node src/bbox_publish_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(rviz_interface_node
  quad_utils
  ${catkin_LIBRARIES}
)

target_link_libraries(terrain_map_publisher_node
  quad_utils
  ${catkin_LIBRARIES}
)

target_link_libraries(trajectory_publisher_node
  quad_utils
  ${catkin_LIBRARIES}
)

target_link_libraries(mesh_to_grid_map_node
  quad_utils
  ${catkin_LIBRARIES}
)

target_link_libraries(remote_heartbeat_node
  quad_utils
  ${catkin_LIBRARIES}
)

target_link_libraries(apply_force_node 
  quad_utils
  ${catkin_LIBRARIES}
)

target_link_libraries(test_bounding_boxes_node
  quad_utils
  ${catkin_LIBRARIES}
)

target_link_libraries(test_collision_checker_node
  quad_utils
  ${catkin_LIBRARIES}
)

target_link_libraries(obstacle_publisher_node
  quad_utils
  ${catkin_LIBRARIES}
)
  target_link_libraries(obstacle_sdf_name_publisher_node
  quad_utils
  ${catkin_LIBRARIES}
)

target_link_libraries(bboxes_node
  quad_utils
  ${catkin_LIBRARIES}
  ${CMAKE_THREAD_LIBS_INIT} # Link the pthread library
)

# add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(quad_utils-test 
  test/test_rviz_interface.cpp
  test/test_terrain_map_publisher.cpp
  test/test_fast_terrain_map.cpp
  test/test_quad_kd.cpp
  test/test_eigen.cpp
  test/test_math_utils.cpp
  test/test_main.cpp)
target_link_libraries(quad_utils-test quad_utils ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})

# Link RBDL libraries to quad_utils-test
target_link_libraries(quad_utils-test
  ${RBDL_LIBRARY}
  ${RBDL_URDFREADER_LIBRARY}
)
