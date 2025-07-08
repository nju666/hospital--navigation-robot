cmake_minimum_required(VERSION 3.5)
project(originbot_face_identify)

# 设置 C++ 标准
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# 编译选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(dnn_node REQUIRED)
find_package(hbm_img_msgs REQUIRED)
find_package(hobot_cv REQUIRED)
find_package(originbot_face_identify_msgs REQUIRED)

# 可选：WebSocket 支持（如需使用 cpp-httplib 或 websocketpp 可放开）
# find_package(ament_cmake_ros REQUIRED)

# 链接 Horizon 平台库（根据地平线实际开发环境调整）
link_directories(
  ${CMAKE_SYSROOT}/usr/lib/hbbpu/
  /usr/hobot/lib
)

# 添加可执行文件
add_executable(originbot_face_identify_node
  src/originbot_face_identify_node.cpp
)

# 链接依赖库
target_link_libraries(originbot_face_identify_node
  hbmem
)

# 添加 include 目录（包括消息包头文件路径）
target_include_directories(originbot_face_identify_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/../originbot_face_identify_msgs/include>
)

# 添加依赖包
ament_target_dependencies(originbot_face_identify_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  OpenCV
  dnn_node
  hbm_img_msgs
  hobot_cv
  originbot_face_identify_msgs
)

# 安装节点
install(TARGETS originbot_face_identify_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装 launch 文件夹（如果有）
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
