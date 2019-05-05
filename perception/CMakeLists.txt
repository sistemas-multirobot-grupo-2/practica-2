cmake_minimum_required(VERSION 2.8.3)
project(perception)

find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	std_msgs
	image_transport
	cv_bridge
	message_generation
	)

add_message_files(FILES
	perception_data.msg
	)

generate_messages(
	DEPENDENCIES
	std_msgs
	)

catkin_package(CATKIN_DEPENDS
	message_runtime
	)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(perception src/perception.cpp)
target_link_libraries(perception ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(perception_tester src/perception_tester.cpp)
target_link_libraries(perception_tester ${catkin_LIBRARIES} ${OpenCV_LIBS})

#First compile with this lines uncommented and then, comment them and re-compile, as said in: http://answers.gazebosim.org/question/4185/error-when-echo-a-rostopic-when-using-gazebo_ros-package-for-ros-intergration-approach/
add_dependencies(perception ${catkin_EXPORTED_TARGETS})
add_dependencies(perception_tester ${catkin_EXPORTED_TARGETS})
