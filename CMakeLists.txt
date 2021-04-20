cmake_minimum_required(VERSION 3.1.2)
project(icp_node)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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
        )

find_package(Eigen3 REQUIRED)
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
set(CMAKE_CXX_FLAGS "")
