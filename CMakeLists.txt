cmake_minimum_required(VERSION 2.8.3)

project(360_Camera_Model)

set(ParentProject "360_Camera_Model")

find_package(catkin REQUIRED COMPONENTS
 	message_generation
	roscpp
	rospy
	std_msgs
	geometry_msgs
	urdf
	xacro
	message_generation
)

catkin_package( 
	CATKIN_DEPENDS
 	geometry_msgs
	roscpp
	rospy
	std_msgs
)

add_subdirectory(${CMAKE_HOME_DIRECTORY}/${ParentProject}/src)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)



