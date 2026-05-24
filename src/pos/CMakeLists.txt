cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0074 NEW)
project(pos LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB_RECURSE SOURCE_FILES "src/*.cpp")
ament_auto_add_library(pos_node SHARED
  ${SOURCE_FILES}
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

target_link_libraries(pos_node
  ${OpenCV_LIBS}
  Eigen3::Eigen
  ${YAML_CPP_LIBRARIES}
)

rclcpp_components_register_node(
  pos_node
  PLUGIN "PosNode"
  EXECUTABLE pos
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE launch config
)
