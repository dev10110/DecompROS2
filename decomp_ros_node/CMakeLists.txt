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

include_directories(
	include
	${PCL_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
	${DECOMP_UTIL_INCLUDE_DIRS}
)

add_executable(decomp_ros_node src/decompRos.cpp)
target_link_libraries(decomp_ros_node
	${PCL_LIBRARIES}
	)

ament_target_dependencies(decomp_ros_node
	rclcpp
	sensor_msgs
	geometry_msgs
	pcl_ros
)

install(TARGETS
	decomp_ros_node
	DESTINATION lib/${PROJECT_NAME})

# ament_export_include_directories(include)

ament_package()

# 
# set(node_plugins "")
# 
# add_library(decompRos_component SHARED
# 	src/decompRos.cpp)
# 
# target_compile_definitions(decompRos_component
# 	PRIVATE "DECOMPROS_COMPOSITION_DLL")
# 
# target_link_libraries(decompRos_component
# 	${sensor_msgs_TARGETS}
# 	)
# 
# ament_target_dependencies(decompRos_component
# 	"rclcpp"
# 	"rclcpp_components"
# 	"sensor_msgs"
# 	)
# 
# rclcpp_components_register_nodes(decompRos_component "decompros::DecompROS")
# 
# set(node_plugins "${node_plugins}decompros::DecompROS;$<TARGET_FILE:decompRos>\n")
# 
# 
# install(TARGETS
# 	decompRos_component
# 	ARCHIVE DESTINATION lib
# 	LIBRARY DESTINATION lib
# 	RUNTIME DESTINATION bin)
# 
# install(DIRECTORY
# 	# launch
# 	DESTINATION share/${PROJECT_NAME}
# 	)
# 
# ament_package()
# 
# 
# 
# 
# 
# 
# 
# # find_package(ament_cmake_auto REQUIRED)
# # ament_auto_find_build_dependencies()
# # 
# # ament_auto_add_library(decompRos SHARED
# # 	src/decompRos.cpp
# # )
# # 
# # # target_link_libraries(decompRos
# # # 	${OCTOMAP_LIBRARIES}
# # #         ${PCL_LIBRARIES}
# # # )
# # 
# # rclcpp_components_register_node(decompRos
# # 	PLUGIN "decompros::DecompROS"
# #   EXECUTABLE decomp_ros_node
# # )
# # 
# # ament_auto_package(
# #   INSTALL_TO_SHARE
# # #     launch
# # #     params
# # )
# # 
# # 
# # install(PROGRAMS
# #   DESTINATION lib/${PROJECT_NAME}
# # )
# # 
# # ament_package()
