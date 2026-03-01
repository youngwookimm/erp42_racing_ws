cmake_minimum_required(VERSION 3.8)
project(erp42_racing_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include /usr/include)

add_library(lla_utm_lib
  src/lla_utm_converter.cpp
)
ament_target_dependencies(lla_utm_lib
  rclcpp
  sensor_msgs
  geometry_msgs
)

target_link_libraries(lla_utm_lib Geographic)

add_executable(lla_utm_node
  src/lla_utm_node.cpp
)
target_link_libraries(lla_utm_node lla_utm_lib)

install(TARGETS
  lla_utm_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS
  src/csv_to_utm_pub.py
  src/utm_origin_shift_node.py
  src/vehicle_pose_viz_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
