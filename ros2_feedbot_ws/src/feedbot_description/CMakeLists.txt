cmake_minimum_required(VERSION 3.8)
project(feedbot_description)

find_package(ament_cmake REQUIRED)

# Install launch files
install(
  DIRECTORY launch urdf config rviz meshes worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install Python executable script
install(PROGRAMS
  feedbot_description/step_test_all_joints.py
  feedbot_description/step_test_joint.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
