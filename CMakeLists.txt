cmake_minimum_required(VERSION 3.8)
project(ct_joy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_executable(ct_joy src/f710_px4_controller.cpp src/joy_speed_limits.cc)
target_compile_features(ct_joy PUBLIC cxx_std_17)
ament_target_dependencies(ct_joy rclcpp sensor_msgs px4_msgs ament_index_cpp)

if(BUILD_TESTING)
  enable_testing()
  add_executable(test_joy_speed_limits test/test_joy_speed_limits.cpp src/joy_speed_limits.cc)
  target_compile_features(test_joy_speed_limits PUBLIC cxx_std_17)
  target_include_directories(test_joy_speed_limits PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  ament_target_dependencies(test_joy_speed_limits rclcpp ament_index_cpp)
  add_test(NAME joy_speed_limit_config COMMAND test_joy_speed_limits)
endif()

install(TARGETS ct_joy
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

ament_package()
