cmake_minimum_required(VERSION 3.16)
project(second_project)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(stage QUIET)
if(NOT stage_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(STAGE REQUIRED stage)
endif()

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

add_executable(goal_publisher src/goal_publisher.cpp)
ament_target_dependencies(goal_publisher
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
)

add_executable(stageros src/stageros.cpp)
ament_target_dependencies(stageros
  rclcpp
  geometry_msgs
  nav_msgs
  rosgraph_msgs
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

if(stage_FOUND)
  target_include_directories(stageros PRIVATE ${STAGE_INCLUDE_DIRS})
  target_link_libraries(stageros ${STAGE_LIBRARIES})
else()
  target_include_directories(stageros PRIVATE ${STAGE_INCLUDE_DIRS})
  target_link_directories(stageros PRIVATE ${STAGE_LIBRARY_DIRS})
  target_link_libraries(stageros ${STAGE_LIBRARIES})
endif()

if(UNIX AND NOT APPLE)
  target_link_libraries(stageros dl)
endif()

install(TARGETS goal_publisher stageros
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config world map csv
  DESTINATION share/${PROJECT_NAME})

ament_package()
