cmake_minimum_required(VERSION 2.8.3)
project(state_filtering)

set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)
add_definitions(-std=c++0x )
add_definitions(-DPROFILING_ON=1) #print profiling output
#set(USE_CUDA True)
add_definitions( -DBUILD_GPU=1 )


find_package(catkin REQUIRED
    roscpp
    roslib
    sensor_msgs
    cv_bridge
    urdf
    orocos_kdl
    kdl_parser
    rosbag
    message_filters
    robot_state_pub
    image_transport
    fl
    dbot
)


set(PROJECT_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
set(CMAKE_MODULE_PATH ${PROJECT_MODULE_PATH})

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED)


set(ccache_DIR ${PROJECT_MODULE_PATH})
find_package(ccache REQUIRED)

if(CCACHE_FOUND)
    message(" ccache found. GCC will be launched via ccache.")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
else(CCACHE_FOUND)
    message(" ccache not found. ")
endif(CCACHE_FOUND)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

#########
## GPU ##
#########
#set(GLEW_DIR ${PROJECT_MODULE_PATH})

#find_package(CUDA)
#find_package(GLEW)
#find_package(OpenGL)

#include_directories(${GLEW_INCLUDE_DIRS})
#include_directories(${OpenGL_INCLUDE_DIRS})
#cuda_include_directories(${CUDA_CUT_INCLUDE_DIRS})

#link_directories(${OpenGL_LIBRARY_DIRS})
#link_directories(${GLEW_LIBRARY_DIRS})

#add_definitions(${OpenGL_DEFINITIONS})
#add_definitions(${GLEW_DEFINITIONS})

#if(NOT CUDA_FOUND)
#    message(" CUDA not found!")
#else(NOT CUDA_FOUND)
#    message(" CUDA has been found")
#endif(NOT CUDA_FOUND)

#if(NOT OPENGL_FOUND)
#    message(" OPENGL not found!")
#else(NOT OPENGL_FOUND)
#    message(" OPENGL has been found!")
#endif(NOT OPENGL_FOUND)

#if(NOT GLEW_FOUND)
#    message(" GLEW not found!")
#else(NOT GLEW_FOUND)
#    message(" GLEW has been found!")
#endif(NOT GLEW_FOUND)



# enable cuda debug information with -g -G -O0, to use with cuda-dbg
# use --ptxas-options=-v to see number of registers, local, shared and constant memory used in kernels
#set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -O2 -arch=sm_20)

##########################################
# Dealing with different assimp versions #
##########################################
set(ASSIMP_PREFIX_PATH ${ASSIMP_HOME} ${ENV_ASSIMP_HOME} /usr/local /usr/local/include /usr/local/lib /usr/include /usr/lib /usr/local/include/assimp /usr/include/assimp /usr/lib/assimp /usr/local/lib/assimp)

# a header only used in Version 3
set(FileToCheckV3 "cimport.h")
find_path(ASSIMP_INCLUDE_DIR_V3
  NAMES ${FileToCheckV3}
  PATHS ${ASSIMP_PREFIX_PATH}
)

# a header only used in Version 2
set(FileToCheckV2 "assimp.h")
find_path(ASSIMP_INCLUDE_DIR_V2
  NAMES ${FileToCheckV2}
  PATHS ${ASSIMP_PREFIX_PATH}
)

IF(EXISTS "${ASSIMP_INCLUDE_DIR_V3}/${FileToCheckV3}")
  message("-- ${FileToCheckV3} exists. Therefore Assimp V3 must be installed and will be used.")
  # add flag for ros distro
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAVE_V3")
ELSEIF(EXISTS "${ASSIMP_INCLUDE_DIR_V2}/${FileToCheckV2}" )
  message("-- ${FileToCheckV2} exists. Therefore Assimp V2 must be installed and will be used.")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHAVE_V2")
ELSE()
   message(FATAL_ERROR "-- Neither ${FileToCheckV2} nor ${FileToCheckV3} exists. Assimp doesn't seem to be installed.")
endif()

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        state_filtering
    CATKIN_DEPENDS
        roscpp
        roslib
        sensor_msgs
        urdf
        orocos_kdl
        kdl_parser
        message_filters
        robot_state_pub
        image_transport
        fl
        dbot
    DEPENDS
        OpenCV
        eigen
        assimp
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} )

set(PROJECT_NAME "state_filtering")

file(GLOB_RECURSE headers include/${PROJECT_NAME}/*.hpp
                          include/${PROJECT_NAME}/*.h)

set(sources
    src/${PROJECT_NAME}/trackers/object_tracker.cpp
    src/${PROJECT_NAME}/trackers/old_object_tracker.cpp
    src/${PROJECT_NAME}/trackers/robot_tracker.cpp

    src/${PROJECT_NAME}/utils/image_visualizer.cpp
    src/${PROJECT_NAME}/utils/cloud_visualizer.cpp
    src/${PROJECT_NAME}/utils/object_file_reader.cpp
    src/${PROJECT_NAME}/utils/kinematics_from_urdf.cpp
    src/${PROJECT_NAME}/utils/tracking_dataset.cpp
    src/${PROJECT_NAME}/utils/robot_tracking_dataset.cpp
    src/${PROJECT_NAME}/utils/pcl_interface.cpp
    src/${PROJECT_NAME}/utils/ros_interface.cpp
    src/${PROJECT_NAME}/utils/image_publisher.cpp)


add_library(${PROJECT_NAME} ${dbot_headers}
                            ${headers}
                            ${sources})


## object_tracker
add_executable(object_tracker
    src/${PROJECT_NAME}/trackers/object_tracker_node.cpp)
target_link_libraries(object_tracker
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    yaml-cpp)


## old_object_tracker
add_executable(old_object_tracker
    src/${PROJECT_NAME}/trackers/old_object_tracker_node.cpp)
target_link_libraries(old_object_tracker
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    yaml-cpp)

## robot_tracker
add_executable(robot_tracker
    src/${PROJECT_NAME}/trackers/robot_tracker_node.cpp)
target_link_libraries(robot_tracker
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    yaml-cpp)

## robot_tracker
add_executable(robot_tracker_evaluation
    src/${PROJECT_NAME}/trackers/robot_tracker_evaluation_node.cpp)
target_link_libraries(robot_tracker_evaluation
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    yaml-cpp)

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES}
                                      ${catkin_LIBRARIES}
                                      ${OpenCV_LIBRARIES}
                                      assimp)




