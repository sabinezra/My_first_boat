cmake_minimum_required(VERSION 3.10)
project(robot_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ros_ign_gazebo REQUIRED)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz config
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

# Install URDF/Xacro files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install meshes if you have them
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
