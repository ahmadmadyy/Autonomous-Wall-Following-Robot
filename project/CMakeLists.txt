cmake_minimum_required(VERSION 3.5)
project(project)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(project_messages REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(topics_node src/topics.cpp)
ament_target_dependencies(topics_node rclcpp geometry_msgs sensor_msgs project_messages)

add_executable(services_node src/server.cpp)
ament_target_dependencies(services_node rclcpp geometry_msgs sensor_msgs project_messages)

add_executable(client_node src/client.cpp)
ament_target_dependencies(client_node rclcpp geometry_msgs sensor_msgs project_messages)

add_executable(action_node src/action.cpp)
ament_target_dependencies(action_node rclcpp geometry_msgs sensor_msgs nav_msgs rclcpp_action project_messages)

add_executable(action_client_node src/action_client.cpp)
ament_target_dependencies(action_client_node rclcpp geometry_msgs sensor_msgs nav_msgs rclcpp_action project_messages)

install(TARGETS
   topics_node
   services_node
   client_node
   action_node
   action_client_node
   DESTINATION lib/${PROJECT_NAME}
 )

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
