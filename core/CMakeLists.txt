cmake_minimum_required(VERSION 3.16)
project(visbot_core LANGUAGES CXX)

# Header-only. Consumed by:
#   * sim/src/visbot_control (ROS 2, via add_subdirectory / ament)
#   * v5/  (PROS build adds core/include to its include path in common.mk)
add_library(visbot_core INTERFACE)
target_include_directories(visbot_core INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(visbot_core INTERFACE cxx_std_17)

option(VISBOT_CORE_TESTS "Build visbot_core unit tests" ON)
if(VISBOT_CORE_TESTS)
    enable_testing()
    foreach(t pid cheesy_drive odometry motion)
        add_executable(test_${t} tests/test_${t}.cpp)
        target_link_libraries(test_${t} PRIVATE visbot_core)
        target_compile_options(test_${t} PRIVATE -Wall -Wextra -Wpedantic)
        add_test(NAME core.${t} COMMAND test_${t})
    endforeach()
endif()

install(DIRECTORY include/ DESTINATION include)

# ---- export so `find_package(visbot_core)` works from the ROS 2 packages ----
include(CMakePackageConfigHelpers)
install(TARGETS visbot_core EXPORT visbot_coreTargets)
install(EXPORT visbot_coreTargets NAMESPACE visbot:: DESTINATION share/visbot_core/cmake)
configure_package_config_file(cmake/visbot_coreConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/visbot_coreConfig.cmake
    INSTALL_DESTINATION share/visbot_core/cmake)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/visbot_coreConfig.cmake DESTINATION share/visbot_core/cmake)
