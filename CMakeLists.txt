cmake_minimum_required(VERSION 3.0.2)
project(imagecontrol)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  pcl_ros
  pcl_conversions
  message_generation
  visualization_msgs
)
catkin_python_setup()


find_package(OpenCV 4.1 REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(Eigen3 REQUIRED)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
    BoundingBox3d.msg
    BoundingBoxes3d.msg
)


add_service_files(
  FILES
  GetNormals.srv
  GetCloud.srv
  
)
## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs std_msgs
 )


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES imagecontrol
 CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs tf pcl_ros message_runtime visualization_msgs
#  DEPENDS system_lib
)


include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
   include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/imagecontrol.cpp
# )
add_executable(imagenode src/image_node.cpp 
                        src/image_process.cpp
                        src/render.cpp
                        src/processPointClouds.cpp
                        src/ransac.cpp
)
add_executable(imagepcd src/pcd_interface.cpp
)    
add_executable(example src/pcl_tutorial.cpp)

add_dependencies(imagenode imagecontrol_generate_messages_cpp)

target_link_libraries(example ${catkin_LIBRARIES})


target_link_libraries(imagenode 
      ${catkin_LIBRARIES}
      ${OpenCV_LIBRARIES}
      ${PCL_LIBRARIES}
)
target_link_libraries(imagepcd
      ${catkin_LIBRARIES}
)


