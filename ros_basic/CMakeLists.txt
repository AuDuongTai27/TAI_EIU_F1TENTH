cmake_minimum_required(VERSION 3.8)
project(ros_basic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosbasic_msgs REQUIRED)
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
  thi_ROS2/node.py

  DESTINATION lib/${PROJECT_NAME})  
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
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
