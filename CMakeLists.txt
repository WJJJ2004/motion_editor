cmake_minimum_required(VERSION 3.12)
project(motion_editor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

#
# Library
#
add_library(${PROJECT_NAME}_lib
  motion_editor/motion_editor.cpp
)

target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_lib
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}_lib
  ament_index_cpp
)

#
# Test executable
#
add_executable(test_node test_code/test.cpp)

target_include_directories(test_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(test_node
  ${PROJECT_NAME}_lib
  yaml-cpp
)

ament_target_dependencies(test_node
  ament_index_cpp
)

#
# Install
#
# 1) Motion YAML files
install(
  DIRECTORY motion
  DESTINATION share/${PROJECT_NAME}
)

# 2) Headers
install(
  DIRECTORY motion_editor/
  DESTINATION include/${PROJECT_NAME}/motion_editor
  FILES_MATCHING PATTERN "*.hpp"
)

# 3) Targets
install(
  TARGETS
    ${PROJECT_NAME}_lib
    test_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
