cmake_minimum_required(VERSION 3.8)
project(cpp_dualsense_twist)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(camera_actuator_interfaces REQUIRED)
find_package(hand_actuator_interfaces REQUIRED)

#link_directories(${CMAKE_SOURCE_DIR}/lib)
message(STATUS "\ndir is ${CMAKE_SOURCE_DIR}\n")


add_library(ds5w SHARED IMPORTED)

# for .lib
set_target_properties(ds5w
	PROPERTIES IMPORTED_IMPLIB
	${CMAKE_SOURCE_DIR}/lib/ds5w_x64.lib)
#	"D:/keita/programming/ros2/windows/dev_ws/src/cpp_dualsense_twist/lib/ds5w_x64.lib")

# for .DLL	
set_target_properties(ds5w
	PROPERTIES IMPORTED_LOCATION
	${CMAKE_SOURCE_DIR}/lib/ds5w_x64.dll)
#	D:/keita/programming/ros2/windows/dev_ws/src/cpp_dualsense_twist/lib/ds5w_x64.dll)


#target_link_libraries(cpp_dualsense_twist PRIVATE ds5w)
#target_link_libraries(talker PRIVATE ds5w)


add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp geometry_msgs camera_actuator_interfaces hand_actuator_interfaces)

target_link_libraries(talker ds5w)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp geometry_msgs camera_actuator_interfaces hand_actuator_interfaces)

install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
