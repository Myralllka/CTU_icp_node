cmake_minimum_required(VERSION 3.1.2)
project(icp_node)

# Override CXX flags inherited from workspace, if precompiled PCL binaries from debian repos are used
if (DEFINED ENV{PCL_CROSS_COMPILATION})
  set(PCL_CROSS_COMPILATION $ENV{PCL_CROSS_COMPILATION})
else()
  set(PCL_CROSS_COMPILATION "false")
endif()
if(${PCL_CROSS_COMPILATION} STREQUAL "false")
  message("Using precompiled PCL binaries from debian repos. Overriding catkin workspace CMAKE_CXX_FLAGS.")
  set(CMAKE_CXX_FLAGS "-std=c++17")
else()
  message("Using custom-built PCL binaries. Inheriting all CMAKE_CXX_FLAGS from catkin workspace.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options("-Wall")
add_compile_options("-Wextra")
# add_compile_options("-DNDEBUG")
add_compile_options("-DEIGEN_INITIALIZE_MATRICES_BY_NAN")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        nodelet
        dynamic_reconfigure
        geometry_msgs
        nav_msgs
        mrs_msgs
        mrs_lib
        pcl_ros
        pcl_conversions
        tf2
        tf2_ros
        tf2_msgs
        tf2_sensor_msgs
        tf2_eigen
        tf2_geometry_msgs
        )

find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
set(Eigen_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
set(Eigen_LIBRARIES ${Eigen_LIBRARIES})

# include PCL
find_package(PCL REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        #  LIBRARIES WaypointFlier
        CATKIN_DEPENDS roscpp mrs_lib mrs_msgs geometry_msgs pcl_ros nav_msgs
        DEPENDS Eigen
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(IcpNode src/IcpNode.cpp include/IcpNode.h)

## Add configure headers for dynamic reconfigure
#add_dependencies(IcpNode ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(
        IcpNode
        ${catkin_LIBRARIES}
        ${Eigen_LIBRARIES}
)

#####################
#Custom
#set(CMAKE_CXX_FLAGS "")
