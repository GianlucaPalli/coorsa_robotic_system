cmake_minimum_required(VERSION 2.8.3)
project(coorsa_interface)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  shape_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf tf2 tf2_bullet tf2_ros tf2_geometry_msgs tf2_kdl tf2_msgs
  moveit_visual_tools
  gazebo_msgs
  actionlib_msgs
  actionlib
  message_generation
  message_runtime
)

find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)
include_directories(include/coorsa_interface)
include_directories(/opt/ros/kinetic/include/kdl_conversions/)

add_action_files(
   FILES
   DetectBoxes.action
   DetectPlanes.action
   EvaluatePlan.action
   PerformPicking.action
)

generate_messages(
   DEPENDENCIES
   actionlib_msgs#   std_msgs
   std_msgs
   geometry_msgs
   shape_msgs
)

add_library(coorsa_interface
  src/coorsa_interface/CoorsaCoordinator.cpp
)

#add_executable(task_manager_test src/task_manager_test.cpp)
#target_link_libraries(task_manager_test ${catkin_LIBRARIES} 
#${Boost_LIBRARIES} ${orocos_kdl_LIBRARIES} coorsa_interface)

add_executable(fake_plane_detector src/fake_plane_detector.cpp)
target_link_libraries(fake_plane_detector ${catkin_LIBRARIES} )

add_executable(fake_box_detector src/fake_box_detector.cpp)
target_link_libraries(fake_box_detector ${catkin_LIBRARIES} )

add_executable(fake_box_planner src/fake_box_planner.cpp)
target_link_libraries(fake_box_planner ${catkin_LIBRARIES} )
