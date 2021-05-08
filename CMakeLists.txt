cmake_minimum_required(VERSION 3.5)
project(realm_ros)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
################################################################################
# Dependencies
################################################################################
find_package(eigen3_cmake_module REQUIRED)
find_package(PCL 1.8 REQUIRED QUIET COMPONENTS common io)
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(CGAL)
find_package (Eigen3 3.3 REQUIRED)
find_package(FLANN 1.7.0 REQUIRED)

find_package(OpenCV 4 REQUIRED)
find_package(OpenREALM REQUIRED)
find_package(openvslam REQUIRED)

include_directories(
  include
  ${PCL_COMMON_INCLUDE_DIRS}
)

include_directories(
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(dependencies
      rclcpp
      rclcpp_components
      pcl_conversions
)

set(INCLUDE_DIRS include 
      ${ament_cmake_INCLUDE_DIRS} 
      ${pcl_conversions_INCLUDE_DIRS})

include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS 
      ${ament_cmake_LIBRARY_DIRS} 
      ${pcl_conversions_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS 
      ${ament_cmake_LIBRARIES} 
      ${geodesy_LIBRARIES}
      ${pcl_conversions_LIBRARIES})


################################################################################
# Build
################################################################################
add_library(${PROJECT_NAME} SHARED 
            src/realm_ros_lib/grabber_exiv2_node.cpp 
            src/realm_ros_lib/stage_node.cpp
)

target_include_directories(${PROJECT_NAME} 
            PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} 
            PUBLIC 
            ${LIBS} 
            ${OpenCV_LIBRARIES} 
            realm_stages)

################################################################################
# Executables
################################################################################

add_executable(realm_stage_node src/stage_node_main.cpp)
target_link_libraries(realm_stage_node PUBLIC ${PROJECT_NAME})

################################################################################
# Install
################################################################################

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib
  RUNTIME
  DESTINATION lib/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h"
  PATTERN "*.hpp")