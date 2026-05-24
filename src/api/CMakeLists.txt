cmake_minimum_required(VERSION 3.8)
project(api)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(vision_msgs REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB_RECURSE SOURCE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg")
rosidl_generate_interfaces(${PROJECT_NAME}
  ${SOURCE_FILES}
  DEPENDENCIES
  std_msgs
  vision_msgs
  geometry_msgs
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
