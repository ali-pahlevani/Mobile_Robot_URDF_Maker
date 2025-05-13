cmake_minimum_required(VERSION 3.5)
project(mobRobURDF_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Install URDF and Mesh files
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Export the package
ament_package()
