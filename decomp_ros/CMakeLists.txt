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


# since the package installs libraries without exporting them
# it needs to make sure that the library path is being exported
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

install(TARGETS
	seedDecomp_component
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
