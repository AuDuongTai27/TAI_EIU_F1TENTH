cmake_minimum_required(VERSION 3.8)
project(ros_basic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
include_directories(
  include
  ${dynamixel_sdk_INCLUDE_DIRS}
)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(rosbasic_msgs REQUIRED)
install(PROGRAMS
  learning_ros/publisher.py    # tao folder: lib , goi ros_publisher.py
  learning_ros/subscriber.py   # tao folder: lib , goi ros_subscriber.py
  learning_ros/subscriberScan.py
  learning_ros/move_robot.py
  learning_ros/service.py
  learning_ros/client1.py
  learning_ros/Interface.py
  tf/static.py
  tf/dynamic.py
  Safety/safetyNode.py
  Safety/brake.py
  Safety/test1.py
  Safety/stop.py
  Safety/AEB.py
  rviz2/bai1.py
  Project1/Project1.py
  Project2/Client.py
  Project2/Service.py
  learning_ros/rv.py
  learning_ros/lidar_to_excel.py
  learning_ros/depth_lidar_capture.py
  learning_ros/depth_lidar_single.py
learning_ros/odom_drive_log.py
learning_ros/servo_steer_log.py

  Review_December/review1.py

  pure_pursuit/controller_real_run.py
  pure_pursuit/controller_simulation.py
  pure_pursuit/simple_simulator.py
  pure_pursuit/reactive_planner_node.py
  pure_pursuit/chay_lay_so_lieu.py

  rrt/rrt.py
  rrt/treeViz.py
  rrt/code_chay_reak.py
  rrt/utils.py
  

  DESTINATION lib/${PROJECT_NAME})  
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
# --- PHẦN C++ ---



add_executable(main_test
  src/main.cpp
  src/pub.cpp
)

ament_target_dependencies(main_test
  rclcpp
  std_msgs
  rcutils
)
# target_link_libraries(main_test ${dynamixel_sdk_LIBRARIES})

## hàm dưới quan trọng
target_include_directories(main_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  src
)
install(TARGETS 
  main_test
  # talker
  DESTINATION lib/${PROJECT_NAME}
)
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
