cmake_minimum_required(VERSION 3.16)
project(visbot_core LANGUAGES CXX)

# The drive controllers, header-only, no dependencies. Used by:
#   * the ROS 2 sim (sim/src/visbot_control, via find_package)
#   * auton_check, the command-line robustness tool built below
#   * v5/, eventually: common.mk puts core/include on the PROS include path,
#     but nothing in v5/src includes it yet.
add_library(visbot_core INTERFACE)
target_include_directories(visbot_core INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
target_compile_features(visbot_core INTERFACE cxx_std_17)

set(VISBOT_AUTONS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../autons")

# ---- auton_check: how repeatable is an auton? ------------------------------
add_executable(auton_check tools/auton_check.cpp)
target_link_libraries(auton_check PRIVATE visbot_core)
target_include_directories(auton_check PRIVATE tools)
target_compile_options(auton_check PRIVATE -Wall -Wextra)
install(TARGETS auton_check RUNTIME DESTINATION bin)

option(VISBOT_CORE_TESTS "Build visbot_core unit tests" ON)
if(VISBOT_CORE_TESTS)
    enable_testing()
    foreach(t pid slew cheesy_drive odometry motion auton_file robot_spec)
        add_executable(test_${t} tests/test_${t}.cpp)
        target_link_libraries(test_${t} PRIVATE visbot_core)
        target_compile_definitions(test_${t} PRIVATE VISBOT_AUTONS_DIR="${VISBOT_AUTONS_DIR}")
        target_compile_options(test_${t} PRIVATE -Wall -Wextra -Wpedantic)
        add_test(NAME core.${t} COMMAND test_${t})
    endforeach()

    # auton_check end to end: every imported routine runs, and the report is
    # deterministic for a given seed.
    add_test(NAME auton_check.all_routines
             COMMAND sh -c "for f in '${VISBOT_AUTONS_DIR}'/*.auton; do $<TARGET_FILE:auton_check> \"$f\" --runs 40 --brief || exit 1; done")
    add_test(NAME auton_check.html
             COMMAND sh -c "$<TARGET_FILE:auton_check> '${VISBOT_AUTONS_DIR}/simple_ring.blue.auton' --runs 50 --html report.html --json report.json && grep -q '\"runs\":50' report.json && ! grep -q 'REPORT_DATA' report.html")
    add_test(NAME auton_check.deterministic
             COMMAND sh -c "a=$($<TARGET_FILE:auton_check> skills --runs 60 --seed 7 --brief); b=$($<TARGET_FILE:auton_check> skills --runs 60 --seed 7 --brief); test \"$a\" = \"$b\"")
    add_test(NAME auton_check.rejects_bad_file
             COMMAND sh -c "printf 'pid_drive_set(1)\\n' > bad.auton && ! $<TARGET_FILE:auton_check> bad.auton --runs 5")
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
