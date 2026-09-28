cmake_minimum_required(VERSION 3.0.2)
project(turtle_drawer)

# catkin 编译所需的 std/c/++ 环境
set(CMAKE_CXX_STANDARD 14)

# 需要包含 ROS 相关头文件和库
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  message_generation
)

# 声明需要生成的消息
add_message_files(
  FILES
  Goal.msg
)

# 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs
)

# 生成 catkin 包
catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs
  message_runtime
)

# 让 Python 脚本可以被直接执行(自动生成可执行权限)
catkin_install_python(PROGRAMS
  scripts/draw_shape.py
  scripts/move_to_goal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)