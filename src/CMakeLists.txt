
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(EXECName robot_info_collector)
set(TestEXECName robot_commander)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_HOME_DIRECTORY}/360_Camera_Model/src
)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_HOME_DIRECTORY}/${EXECName})


add_executable(${EXECName} robot_info_collector.cpp)
target_link_libraries(${EXECName}
  ${catkin_LIBRARIES}
)
add_executable(${TestEXECName} robot_commander.cpp)
target_link_libraries(${TestEXECName}
  ${catkin_LIBRARIES}
)
