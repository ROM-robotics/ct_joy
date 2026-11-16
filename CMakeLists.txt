cmake_minimum_required(VERSION 3.8)
project(ct_joy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(ct_joy src/f710_px4_controller.cpp)
target_compile_features(ct_joy PUBLIC cxx_std_17)
ament_target_dependencies(ct_joy rclcpp sensor_msgs px4_msgs)

install(TARGETS ct_joy
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(FILES README.md
  DESTINATION share/${PROJECT_NAME})

ament_package()
