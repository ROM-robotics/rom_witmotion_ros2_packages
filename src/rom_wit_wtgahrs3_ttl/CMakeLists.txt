cmake_minimum_required(VERSION 3.8)
project(rom_wit_wtgahrs3_ttl)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(wit_wtgahrs3_ttl
  src/wit_wtgahrs3_ttl.cpp
  src/serial.c
  src/wit_c_sdk.c
)
target_include_directories(wit_wtgahrs3_ttl PRIVATE src)
ament_target_dependencies(wit_wtgahrs3_ttl
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  wit_wtgahrs3_ttl
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
