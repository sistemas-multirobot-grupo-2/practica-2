cmake_minimum_required(VERSION 2.8.3)
project(a_multirob)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(perception src/perception.cpp)
target_link_libraries(perception ${catkin_LIBRARIES} ${OpenCV_LIBS})
