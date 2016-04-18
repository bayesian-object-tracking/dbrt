cmake_minimum_required(VERSION 2.8.3)
project(dbrt)

############################
# Options                  #
############################
option(DBOT_BUILD_GPU "Compile CUDA enabled trackers" ON)

find_package(CUDA QUIET)
if(DBOT_BUILD_GPU AND CUDA_FOUND)
  # activate gpu implementations
  add_definitions(-DDBOT_BUILD_GPU=1)
  set(DBOT_GPU_SUPPORT "YES")
else(DBOT_BUILD_GPU AND CUDA_FOUND AND GLEW_FOUND AND OPENGL_FOUND)
  set(DBOT_GPU_SUPPORT "NO")
  if(DBOT_BUILD_GPU)
    message(WARNING "No CUDA support. Deactivating GPU implementation")
  endif(DBOT_BUILD_GPU)
  set(DBOT_BUILD_GPU OFF)
endif(DBOT_BUILD_GPU AND CUDA_FOUND AND GLEW_FOUND AND OPENGL_FOUND)

add_definitions(-std=c++0x -fno-omit-frame-pointer)
add_definitions(-DPROFILING_ON=1) #print profiling output

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
    osr
    dbot_ros
)

set(PROJECT_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
set(CMAKE_MODULE_PATH ${PROJECT_MODULE_PATH})

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(PCL 1.3 REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})
#include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

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
        dbrt
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
        osr
        dbot
        dbot_ros
    DEPENDS
        OpenCV
        eigen
        assimp
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE headers include/${PROJECT_NAME}/*.hpp
                          include/${PROJECT_NAME}/*.h)

set(sources
    src/${PROJECT_NAME}/robot_publisher.cpp
    src/${PROJECT_NAME}/robot_tracker.cpp
    src/${PROJECT_NAME}/fusion_tracker.cpp
    src/${PROJECT_NAME}/visual_tracker.cpp
    src/${PROJECT_NAME}/rotary_tracker.cpp
    src/${PROJECT_NAME}/robot_tracker_node.cpp
    src/${PROJECT_NAME}/util/builder/visual_tracker_builder.cpp
    src/${PROJECT_NAME}/util/builder/visual_tracker_factory.cpp
    src/${PROJECT_NAME}/util/builder/robot_rb_observation_model_builder.cpp
    src/${PROJECT_NAME}/util/urdf_object_loader.cpp
    src/${PROJECT_NAME}/util/image_visualizer.cpp
    src/${PROJECT_NAME}/util/kinematics_from_urdf.cpp
    src/${PROJECT_NAME}/util/robot_tracking_dataset.cpp
    src/${PROJECT_NAME}/util/image_publisher.cpp
    src/${PROJECT_NAME}/util/robot_transformer.cpp)


add_library(${PROJECT_NAME} ${dbot_headers}
                            ${headers}
                            ${sources})

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES}
                                      ${catkin_LIBRARIES}
                                      ${OpenCV_LIBRARIES}
                                      assimp)

# robot_tracker
add_executable(visual_tracker
     src/${PROJECT_NAME}/visual_tracker_node.cpp)
target_link_libraries(visual_tracker
     ${PROJECT_NAME}
     ${catkin_LIBRARIES}
     ${OpenCV_LIBS}
     ${PCL_LIBRARIES}
     yaml-cpp)

add_executable(rotary_tracker
     src/${PROJECT_NAME}/rotary_tracker_node.cpp)
target_link_libraries(rotary_tracker
     ${PROJECT_NAME}
     ${catkin_LIBRARIES}
     ${OpenCV_LIBS}
     ${PCL_LIBRARIES}
     yaml-cpp)

add_executable(visual_tracker_offline
     src/${PROJECT_NAME}/visual_tracker_offline_node.cpp)
target_link_libraries(visual_tracker_offline
     ${PROJECT_NAME}
     ${catkin_LIBRARIES}
     ${OpenCV_LIBS}
     ${PCL_LIBRARIES}
     yaml-cpp)

add_executable(arm_robot_emulator
     src/${PROJECT_NAME}/arm_robot_emulator_node.cpp)
target_link_libraries(arm_robot_emulator
     ${PROJECT_NAME}
     ${catkin_LIBRARIES}
     ${OpenCV_LIBS}
     ${PCL_LIBRARIES}
     yaml-cpp)

add_executable(fusion_tracker
     src/${PROJECT_NAME}/fusion_tracker_node.cpp)
target_link_libraries(fusion_tracker
     ${PROJECT_NAME}
     ${catkin_LIBRARIES}
     ${OpenCV_LIBS}
     ${PCL_LIBRARIES}
     yaml-cpp)
