cmake_minimum_required(VERSION 2.8.3)
project(wall_follow)

find_package(catkin REQUIRED
std_msgs
roscpp
geometry_msgs
sensor_msgs
)

catkin_package(
CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(example src/example.cpp)
target_link_libraries(example  ${catkin_LIBRARIES})
add_dependencies(example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})