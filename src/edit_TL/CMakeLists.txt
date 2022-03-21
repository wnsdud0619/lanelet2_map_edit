cmake_minimum_required(VERSION 3.0.2)
project(edit_TL)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS 
  message_generation 
  sensor_msgs 
  roscpp 
  cv_bridge 
  image_transport 
  autoware_perception_msgs 
  message_filters
  tf
  tf2 
  tf2_ros
  tf2_eigen
  lanelet2_extension
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

generate_messages(DEPENDENCIES sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  autoware_perception_msgs
  tf
  tf2
  tf2_ros
  tf2_eigen
  lanelet2_extension
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} 
  src/node.cpp
  src/main.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
    )
