cmake_minimum_required(VERSION 3.5)
project(mobRobURDF_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Export the package
ament_package()
