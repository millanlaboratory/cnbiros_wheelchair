set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_wheelchair)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation  nav_msgs geometry_msgs cnbiros_core)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)


add_service_files(
  FILES
  Stop.srv
  Forward.srv
  SetVelocity.srv
  GetVelocity.srv
)

generate_messages(
	DEPENDENCIES
	std_msgs
)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
	message_runtime
)

include_directories(${catkin_INCLUDE_DIRS} 
		    include)

add_library(cnbiros_wheelchair
			src/DXGPSB.cpp
			src/DXGPSBThread.cpp
			src/Motors.cpp)
target_link_libraries(cnbiros_wheelchair
					  ${catkin_LIBRARIES})

add_executable(example_dxgpsb examples/example_dxgpsb.cpp)
add_dependencies(example_dxgpsb cnbiros_wheelchair_generate_messages_cpp)
target_link_libraries(example_dxgpsb cnbiros_wheelchair)

add_executable(ros_wheelchair_motors src/ros_wheelchair_motors.cpp)
add_dependencies(ros_wheelchair_motors cnbiros_wheelchair_generate_messages_cpp)
target_link_libraries(ros_wheelchair_motors cnbiros_wheelchair)
