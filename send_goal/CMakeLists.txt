cmake_minimum_required(VERSION 3.5)
project(originbot_send_goal)


if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(send_goal_node src/send_goal_node.cpp)

ament_target_dependencies(send_goal_node
  rclcpp
  rclcpp_action
  std_msgs
  nav2_msgs
)

install(TARGETS
  send_goal_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY goal_pose
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

