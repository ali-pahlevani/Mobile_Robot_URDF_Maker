cmake_minimum_required(VERSION 3.5)
project(mobRobURDF_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_bt_navigator REQUIRED)
find_package(nav2_lifecycle_manager REQUIRED)

# Install config files
install(
  DIRECTORY config maps
  DESTINATION share/${PROJECT_NAME}
)

# Export the package
ament_package()
