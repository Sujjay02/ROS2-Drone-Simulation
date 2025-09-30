cmake_minimum_required(VERSION 3.8)
project(sweeping_generator)

# Find dependencies.
# ament_cmake is the primary build tool.
# rosidl_default_generators is needed to build .srv/.msg files.
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(mrs_msgs REQUIRED) # Assuming your .srv files might depend on this

# This single command replaces 'add_service_files' and 'generate_messages'.
# It generates the necessary C++ and Python code for your services.
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Path.srv"
  "srv/Start.srv"
  # Add any message packages your .srv files depend on here.
  DEPENDENCIES mrs_msgs
)

# Install the package manifest file.
install(
  FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install the Python node script to the package's library directory.
install(
  PROGRAMS scripts/sweeping_generator.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch directory.
# Remember to convert your .launch file to a Python launch file.
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/
)

# This final command is the equivalent of catkin_package().
# It registers the package with the ament index, making it discoverable.
ament_package()