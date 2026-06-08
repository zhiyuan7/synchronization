cmake_minimum_required(VERSION 3.8)
project(sync)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(OpenCV REQUIRED)

file(GLOB_RECURSE SOURCE_FILES "src/*.cpp")
ament_auto_add_library(sync_node SHARED
  ${SOURCE_FILES}
)

target_link_libraries(sync_node ${OpenCV_LIBS})

rclcpp_components_register_node(
  sync_node
  PLUGIN "SyncNode"
  EXECUTABLE sync
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE launch config
)
