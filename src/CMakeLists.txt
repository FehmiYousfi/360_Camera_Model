cmake_minimum_required(VERSION 3.0.2)
project(360_Camera_Model)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  roslib 
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_HOME_DIRECTORY}/360_Camera_Model/src
)

# Set the output directory for executables
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/${PROJECT_NAME}/bin)

# Collect all C++ source files in the src directory
file(GLOB SRC_FILES "${CMAKE_HOME_DIRECTORY}/360_Camera_Model/src/*.cpp")

# Iterate over each source file and create an executable
foreach(SRC_FILE ${SRC_FILES})
  get_filename_component(EXEC_NAME ${SRC_FILE} NAME_WE)  # Get the base name without extension
  add_executable(${EXEC_NAME} ${SRC_FILE})  # Create executable
  target_link_libraries(${EXEC_NAME} ${catkin_LIBRARIES})  # Link libraries
endforeach()

# Optional: If you want to keep the previous hardcoded executables
# add_executable(robot_info_collector robot_info_collector.cpp)
# target_link_libraries(robot_info_collector ${catkin_LIBRARIES})

# add_executable(robot_commander robot_commander.cpp)
# target_link_libraries(robot_commander ${catkin_LIBRARIES})

