set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_wheelchair)

find_package(catkin REQUIRED COMPONENTS roscpp message_generation std_msgs sensor_msgs nav_msgs geometry_msgs cnbiros_core tf)

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
			src/Dxgpsb.cpp
			src/DxgpsbThread.cpp
			src/Motors.cpp
			src/Encoder.cpp
			src/EncoderThread.cpp
			src/OdometryThread.cpp
			src/Odometry.cpp
			src/Sonar.cpp
			src/SonarThread.cpp
			src/UltraSonics.cpp)
add_dependencies(cnbiros_wheelchair cnbiros_wheelchair_generate_messages_cpp)
target_link_libraries(cnbiros_wheelchair
					  ${catkin_LIBRARIES})

add_executable(example_dxgpsb examples/example_dxgpsb.cpp)
add_dependencies(example_dxgpsb cnbiros_wheelchair_generate_messages_cpp)
target_link_libraries(example_dxgpsb cnbiros_wheelchair)

add_executable(example_sonars examples/example_sonars.cpp)
add_dependencies(example_sonars cnbiros_wheelchair_generate_messages_cpp)
target_link_libraries(example_sonars cnbiros_wheelchair)

add_executable(example_sonars_setaddress examples/example_sonars_setaddress.cpp)
add_dependencies(example_sonars_setaddress cnbiros_wheelchair_generate_messages_cpp)
target_link_libraries(example_sonars_setaddress cnbiros_wheelchair)

add_executable(example_encoder_pair examples/example_encoder_pair.cpp)
add_dependencies(example_encoder_pair cnbiros_wheelchair_generate_messages_cpp)
target_link_libraries(example_encoder_pair cnbiros_wheelchair)

add_executable(example_encoder_single examples/example_encoder_single.cpp)
add_dependencies(example_encoder_single cnbiros_wheelchair_generate_messages_cpp)
target_link_libraries(example_encoder_single cnbiros_wheelchair)

add_executable(example_odometry examples/example_odometry.cpp)
add_dependencies(example_odometry cnbiros_wheelchair_generate_messages_cpp)
target_link_libraries(example_odometry cnbiros_wheelchair)

add_executable(ros_wheelchair_motors src/ros_wheelchair_motors.cpp)
add_dependencies(ros_wheelchair_motors cnbiros_wheelchair_generate_messages_cpp)
target_link_libraries(ros_wheelchair_motors cnbiros_wheelchair)

add_executable(ros_wheelchair_odometry src/ros_wheelchair_odometry.cpp)
add_dependencies(ros_wheelchair_odometry cnbiros_wheelchair_generate_messages_cpp)
target_link_libraries(ros_wheelchair_odometry cnbiros_wheelchair)

add_executable(ros_wheelchair_ultrasonics src/ros_wheelchair_ultrasonics.cpp)
add_dependencies(ros_wheelchair_ultrasonics cnbiros_wheelchair_generate_messages_cpp)
target_link_libraries(ros_wheelchair_ultrasonics cnbiros_wheelchair)
