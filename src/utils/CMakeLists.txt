cmake_minimum_required(VERSION 3.8)
project(utils)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB_RECURSE SOURCE_FILES "src/*.cpp")
ament_auto_add_library(utils SHARED
  ${SOURCE_FILES}
)

find_package(OpenCV REQUIRED)
target_link_libraries(utils
  ${OpenCV_LIBS}
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
