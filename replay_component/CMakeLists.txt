cmake_minimum_required(VERSION 3.0.2)
project(replay_component)

set(CMAKE_CXX_STANDARD 14)
set(OpenCV_DIR /usr/share/OpenCV/)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  geometry_msgs
  tf2_ros
  rosbag
  cv_bridge
  image_transport
  OpenCV
  serial
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(Node1 src/Node1.cpp)
target_link_libraries(Node1 ${catkin_LIBRARIES} )
add_dependencies(Node1 replay_component_generate_messages_cpp)

add_executable(Node5 src/Node5.cpp)
target_link_libraries(Node5 ${catkin_LIBRARIES})
add_dependencies(Node5 replay_component_generate_messages_cpp)

add_executable(Node6 src/Node6.cpp)
target_link_libraries(Node6 ${catkin_LIBRARIES} )
add_dependencies(Node6 replay_component_generate_messages_cpp)

add_executable(Node3 src/Node3.cpp src/ADAS_Support.c)
target_link_libraries(Node3 ${catkin_LIBRARIES} )
add_dependencies(Node3 replay_component_generate_messages_cpp)

add_executable(Node2 src/Node2.cpp )
target_link_libraries(Node2 ${catkin_LIBRARIES} )
add_dependencies(Node2 replay_component_generate_messages_cpp)

