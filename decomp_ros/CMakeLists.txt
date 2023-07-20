cmake_minimum_required(VERSION 3.5)
project(decomp_ros)

if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 14)
	set(CMAKE_CXX_STANDARD_REQUIRED ON)
	set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release")
	# set(CMAKE_BUILD_TYPE "Debug")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(decomp_util REQUIRED)
find_package(std_msgs REQUIRED)
find_package(decomp_ros_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


if(TARGET tf2_geometry_msgs::tf2_geometry_msgs)
  get_target_property(_include_dirs tf2_geometry_msgs::tf2_geometry_msgs INTERFACE_INCLUDE_DIRECTORIES)
else()
  set(_include_dirs ${tf2_geometry_msgs_INCLUDE_DIRS})
endif()

find_file(TF2_CPP_HEADERS
  NAMES tf2_geometry_msgs.hpp
  PATHS ${_include_dirs}
  NO_CACHE
  PATH_SUFFIXES tf2_geometry_msgs
)



include_directories(
	include
	${PCL_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	${DECOMP_UTIL_INCLUDE_DIRS}
)



set(node_plugins "")

add_library(seedDecomp_component SHARED
	src/decomp_ros_component.cpp
)
target_compile_definitions(seedDecomp_component 
	PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(seedDecomp_component
	${PCL_LIBRARIES}
	)
ament_target_dependencies(seedDecomp_component
	rclcpp
	rclcpp_components
	std_msgs
	sensor_msgs
	geometry_msgs
	pcl_ros
	decomp_ros_msgs
	)

rclcpp_components_register_node(seedDecomp_component
	PLUGIN "decompros::SeedDecomp"
	EXECUTABLE seedDecomp_node)

set(node_plugins "${node_plugins}decompros::SeedDecomp;$<TARGET_FILE:seedDecomp_component>\n")






add_library(vizPoly_component SHARED
	src/visualize_polyhedron.cpp
)
target_compile_definitions(vizPoly_component 
	PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(vizPoly_component
	${PCL_LIBRARIES}
	)
ament_target_dependencies(vizPoly_component
	rclcpp
	rclcpp_components
	std_msgs
	geometry_msgs
	decomp_ros_msgs
	visualization_msgs
	)

rclcpp_components_register_node(vizPoly_component
	PLUGIN "decompros::VizPoly"
	EXECUTABLE vizPoly_node)

set(node_plugins "${node_plugins}decompros::VizPoly;$<TARGET_FILE:vizPoly_component>\n")






add_library(sfcBuffer_component SHARED
	src/sfc_buffer.cpp
)
target_compile_definitions(sfcBuffer_component 
	PRIVATE "COMPOSITION_BUILDING_DLL")
target_link_libraries(sfcBuffer_component
  ${PCL_LIBRARIES}
)
ament_target_dependencies(sfcBuffer_component
	rclcpp
	rclcpp_components
	geometry_msgs
	decomp_ros_msgs
	message_filters
	tf2_ros
	tf2_geometry_msgs
	)
if(EXISTS ${TF2_CPP_HEADERS})
  target_compile_definitions(sfcBuffer_component PUBLIC -DTF2_CPP_HEADERS)
endif()

rclcpp_components_register_node(sfcBuffer_component
	PLUGIN "decompros::SFCBuffer"
	EXECUTABLE sfcBuffer_node)

set(node_plugins "${node_plugins}decompros::SFCBuffer;$<TARGET_FILE:sfcBuffer_component>\n")






# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(TARGETS
	seedDecomp_component
	vizPoly_component
	sfcBuffer_component
	ARCHIVE DESTINATION lib
	LIBRARY DESTINATION lib
	RUNTIME DESTINATION lib
	)



# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
