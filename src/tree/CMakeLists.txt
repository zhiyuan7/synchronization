cmake_minimum_required(VERSION 3.8)
project(tree)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB_RECURSE SOURCE_FILES "src/*.cpp")
ament_auto_add_library(tree_node SHARED
  ${SOURCE_FILES}
)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
target_link_libraries(tree_node
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
)

rclcpp_components_register_node(
  tree_node
  PLUGIN "TreeNode"
  EXECUTABLE tree
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE launch config
)
