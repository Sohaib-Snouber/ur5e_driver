cmake_minimum_required(VERSION 3.8)
project(master_ur5e_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(ur_rtde REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(master_project_msgs)
find_package(moveit_visual_tools REQUIRED)
find_package(rclcpp_action REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${moveit_task_constructor_core_INCLUDE_DIRS}
  ${ur_rtde_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

add_executable(task_action_server src/task_action_server.cpp)
ament_target_dependencies(task_action_server 
  rclcpp 
  rclcpp_action 
  master_project_msgs 
  moveit_core 
  moveit_task_constructor_core 
  moveit_ros_planning_interface
)


add_executable(planning_scene_node src/planning_scene_node.cpp)
ament_target_dependencies(planning_scene_node
 rclcpp 
 moveit_core 
 moveit_ros_planning_interface 
 moveit_task_constructor_core 
 ur_rtde
)

add_executable(target_pose_node src/target_pose_node.cpp)
ament_target_dependencies(target_pose_node
 rclcpp 
 moveit_core 
 moveit_ros_planning_interface 
 moveit_task_constructor_core 
 ur_rtde
)

add_executable(task_execution_node src/task_execution_node.cpp)
ament_target_dependencies(task_execution_node
 rclcpp moveit_core
 moveit_ros_planning_interface 
 moveit_task_constructor_core 
 ur_rtde 
 geometry_msgs   
 master_project_msgs
)

add_executable(sync_rviz_pose src/sync_rviz_pose.cpp)
ament_target_dependencies(sync_rviz_pose
  moveit_ros_planning_interface
  moveit_visual_tools
  rclcpp
  geometry_msgs
  std_msgs
  moveit_msgs
  moveit_core
  ur_rtde
)
add_executable(testing_server src/testing_server.cpp)
ament_target_dependencies(testing_server
 rclcpp 
 rclcpp_action
 master_project_msgs
)

install(TARGETS 
  sync_rviz_pose
  planning_scene_node
  target_pose_node
  task_execution_node
  task_action_server
  testing_server
  DESTINATION lib/${PROJECT_NAME})

target_link_libraries(task_action_server
  ur_rtde::rtde
)
target_link_libraries(sync_rviz_pose
  ur_rtde::rtde
)

install(DIRECTORY launch
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
