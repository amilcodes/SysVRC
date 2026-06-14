cmake_minimum_required(VERSION 3.16)
project(visbot_control LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visbot_msgs REQUIRED)
find_package(visbot_core REQUIRED)
find_package(Threads REQUIRED)

add_library(rt_loop INTERFACE)
target_include_directories(rt_loop INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(rt_loop INTERFACE Threads::Threads)

add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node PRIVATE rt_loop visbot::visbot_core
  rclcpp::rclcpp ${sensor_msgs_TARGETS} ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS} ${visbot_msgs_TARGETS})

add_executable(plant_node src/plant_node.cpp)
target_link_libraries(plant_node PRIVATE rt_loop visbot::visbot_core
  rclcpp::rclcpp ${sensor_msgs_TARGETS} ${geometry_msgs_TARGETS} ${nav_msgs_TARGETS})

# No ROS dependency: this is the thing you run to get the latency numbers.
add_executable(rt_bench src/rt_bench.cpp)
target_link_libraries(rt_bench PRIVATE rt_loop visbot::visbot_core)

install(TARGETS controller_node plant_node rt_bench DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rt_loop test/test_rt_loop.cpp)
  target_link_libraries(test_rt_loop rt_loop)
endif()

ament_package()
