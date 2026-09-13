cmake_minimum_required(VERSION 3.10)
project(pb_serial_comm)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_library(serial_port SHARED
  src/serial_port.cpp
)
target_include_directories(serial_port PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(serial_port Boost::system)

add_library(frame SHARED
  src/frame.cpp
)
target_include_directories(frame PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(serial_comm_node
  src/serial_comm_node.cpp
)
target_include_directories(serial_comm_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(serial_comm_node
  serial_port
  frame
)
ament_target_dependencies(serial_comm_node
  rclcpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  example_interfaces
)

install(TARGETS
  serial_port
  frame
  serial_comm_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
