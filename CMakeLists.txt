cmake_minimum_required(VERSION 3.0.2)
project(offboard)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2
  tf2_eigen
  cv_bridge
  image_transport
  mavros_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(offboard_test_node src/offboard_test_node.cpp)
target_link_libraries(offboard_test_node ${catkin_LIBRARIES})

add_executable(figure_8_circle_node src/figure_8_circle_node.cpp)
target_link_libraries(figure_8_circle_node ${catkin_LIBRARIES})

add_executable(vins_to_mavros_node src/vision_to_mavros.cpp)
target_link_libraries(vins_to_mavros_node ${catkin_LIBRARIES})