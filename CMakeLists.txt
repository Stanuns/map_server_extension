cmake_minimum_required(VERSION 3.8)
project(map_server_extension)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)

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

add_executable(map_saver_client src/map_saver_client.cpp)
ament_target_dependencies(map_saver_client
  robot_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_msgs
  slam_toolbox
)
add_executable(map_mgmt_server src/map_mgmt_server.cpp)
ament_target_dependencies(map_mgmt_server
  robot_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_msgs
  std_srvs
  nav_msgs
)
target_link_libraries(map_mgmt_server
  ${YAML_CPP_LIBRARIES}
)

add_executable(get_map_file_client_test src/get_map_file_client_test.cpp)
ament_target_dependencies(get_map_file_client_test
  robot_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_msgs
  std_srvs
)
target_link_libraries(get_map_file_client_test
  ${YAML_CPP_LIBRARIES}
)

add_executable(update_map_file_client_test src/update_map_file_client_test.cpp)
ament_target_dependencies(update_map_file_client_test
  robot_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  nav2_msgs
  std_srvs
)
target_link_libraries(update_map_file_client_test
  ${YAML_CPP_LIBRARIES}
)


install(TARGETS map_saver_client map_mgmt_server get_map_file_client_test update_map_file_client_test
    DESTINATION lib/${PROJECT_NAME}
)
install(
        DIRECTORY  maps params
        DESTINATION share/${PROJECT_NAME}
)

ament_package()
