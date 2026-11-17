cmake_minimum_required(VERSION 3.16)
project(rmf_simulation_lift_controller)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmf_lift_msgs REQUIRED)

# Lift selection logic, independent from the ROS node so it can be unit tested
add_library(lift_allocator STATIC
  src/LiftAllocator.cpp
)
target_include_directories(lift_allocator
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(lift_allocator
  PUBLIC
    ${rmf_lift_msgs_TARGETS}
)

add_executable(simulated_lift_controller
  src/main.cpp
  src/SimulatedLiftController.cpp
)
target_link_libraries(simulated_lift_controller
  PRIVATE
    lift_allocator
    rclcpp::rclcpp
    ${rmf_lift_msgs_TARGETS}
)

install(TARGETS simulated_lift_controller
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_lift_allocator test/test_lift_allocator.cpp)
  target_link_libraries(test_lift_allocator lift_allocator)
endif()

ament_package()
