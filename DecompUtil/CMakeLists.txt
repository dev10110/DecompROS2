# cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.5)
project(decomp_util)

# set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-deprecated-declarations")

set(BUILD_SHARED_LIBS ON)

find_package(Eigen3 3.3.7 EXACT REQUIRED) # v3.4 is not allowed
find_package(Boost REQUIRED)

# # Add Eigen include directories
if (Eigen3_FOUND)
    message("Found Eigen version: ${Eigen3_VERSION}")
    message("Eigen found: ${EIGEN3_INCLUDE_DIRS}")
else()
    message(FATAL_ERROR "Eigen not found.")
endif()

# if (Boost_FOUND)
#     message("Found Boost version: ${Boost_VERSION}")
#     message("Boost Libraries: ${Boost_LIBRARIES}")
# else()
#     message(FATAL_ERROR "Boost not found.")
# endif()

include_directories(${EIGEN3_INCLUDE_DIRS} include)

include(CTest)


# if (BUILD_TESTING)
# 
#   add_executable(test_seed_decomp test/test_seed_decomp.cpp)
#   target_link_libraries(test_seed_decomp ${Boost_LIBRARIES})
#   add_test(test_seed_decomp test_seed_decomp)
#   
#   add_executable(test_line_segment test/test_line_segment.cpp)
#   target_link_libraries(test_line_segment ${Boost_LIBRARIES})
#   add_test(test_line_segment test_line_segment ${CMAKE_SOURCE_DIR}/data/obstacles.txt)
#   
#   add_executable(test_ellipsoid_decomp test/test_ellipsoid_decomp.cpp)
#   target_link_libraries(test_ellipsoid_decomp ${Boost_LIBRARIES})
#   add_test(test_ellipsoid_decomp test_ellipsoid_decomp ${CMAKE_SOURCE_DIR}/data/obstacles.txt)
#   
#   add_executable(test_iterative_decomp test/test_iterative_decomp.cpp)
#   target_link_libraries(test_iterative_decomp ${Boost_LIBRARIES})
#   add_test(test_iterative_decomp test_iterative_decomp ${CMAKE_SOURCE_DIR}/data/obstacles.txt)
#   
#   add_executable(test_line_segment_3d test/test_line_segment_3d.cpp)
# 
# endif()

install(FILES "${PROJECT_NAME}Config.cmake" "${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION "share/${PROJECT_NAME}/cmake")

install(TARGETS
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/decomp_basis include/decomp_geometry include/decomp_util
  DESTINATION include)
