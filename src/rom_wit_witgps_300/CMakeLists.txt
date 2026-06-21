cmake_minimum_required(VERSION 3.8)
project(rom_wit_witgps_300)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enable verbose RCLCPP_INFO debug output of every decoded NMEA field:
#   colcon build --cmake-args -DROM_DEBUG=ON
option(ROM_DEBUG "Enable verbose GNSS debug logging" OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Custom messages (satellite + DOP status)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SatelliteInfo.msg"
  "msg/WitGpsStatus.msg"
  DEPENDENCIES std_msgs
)

# Vendored minmea NMEA-0183 parser (third_party/minmea)
add_library(minmea STATIC third_party/minmea/minmea.c)
target_include_directories(minmea PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/minmea>")
# minmea uses timegm(), which needs the default/BSD feature set on glibc.
target_compile_definitions(minmea PRIVATE _DEFAULT_SOURCE)
set_property(TARGET minmea PROPERTY POSITION_INDEPENDENT_CODE ON)

# Driver node
add_executable(wit_gps_300_ttl src/wit_gps_300_ttl.cpp)
target_include_directories(wit_gps_300_ttl PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/minmea")
ament_target_dependencies(wit_gps_300_ttl
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)
target_link_libraries(wit_gps_300_ttl minmea)

if(ROM_DEBUG)
  target_compile_definitions(wit_gps_300_ttl PRIVATE ROM_DEBUG)
endif()

# Link the messages generated in this same package.
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(wit_gps_300_ttl "${cpp_typesupport_target}")

install(TARGETS wit_gps_300_ttl
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

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

ament_package()
