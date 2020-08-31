cmake_minimum_required(VERSION 2.8.3)
project(auto_dock)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  laser_line_extraction
  visualization_msgs
  geometry_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pattern
#  CATKIN_DEPENDS roscpp geometry_msgs visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(pattern src/pattern.cpp)
add_executable(controller src/controller.cpp)

add_dependencies(pattern ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pattern
  ${catkin_LIBRARIES}
)

target_link_libraries(controller
  ${catkin_LIBRARIES}
)