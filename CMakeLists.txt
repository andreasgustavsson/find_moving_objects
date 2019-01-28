cmake_minimum_required(VERSION 2.8.3)
project(find_moving_objects)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp 
  nodelet
  tf
#   topic_tools
  message_generation 
  sensor_msgs 
  visualization_msgs 
  geometry_msgs
)

# find_package(OpenMP REQUIRED)
# if(OPENMP_FOUND)
#   message(STATUS "OPENMP FOUND")
#   set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
#   set(OpenMP_LIBS gomp)
# else()
#   message(STATUS "OPENMP NOT FOUND")
# endif()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED)

# Use features from C++ 11
if(NOT WIN32)
  add_compile_options(-std=c++11)
endif()

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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder
add_message_files(
  FILES 
  MovingObject.msg 
  MovingObjectArray.msg 
  PointCloud2Array.msg
  LaserScanArray.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES 
  geometry_msgs 
  sensor_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES find_moving_objects
 CATKIN_DEPENDS roscpp 
                nodelet
                tf 
                message_runtime 
                sensor_msgs 
                visualization_msgs 
                geometry_msgs 
 DEPENDS Boost
#          OpenMP
)

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
# add_library(option  src/${PROJECT_NAME}/option.cpp)
# add_library(hz_calculator  src/${PROJECT_NAME}/hz_calculator.cpp)
add_library(find_moving_objects  src/${PROJECT_NAME}/bank.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(find_moving_objects ${PROJECT_NAME}_generate_messages)
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
add_executable(laserscan_interpreter_node src/laserscan_interpreter.cpp)
add_library(LaserScanInterpreterNodelet src/laserscan_interpreter.cpp)
add_executable(laserscanarray_interpreter_node src/laserscan_interpreter.cpp)
add_library(LaserScanArrayInterpreterNodelet src/laserscan_interpreter.cpp)
add_executable(pointcloud2_interpreter_node src/pointcloud2_interpreter.cpp)
add_library(PointCloud2InterpreterNodelet src/pointcloud2_interpreter.cpp)
add_executable(pointcloud2array_interpreter_node src/pointcloud2_interpreter.cpp)
add_library(PointCloud2ArrayInterpreterNodelet src/pointcloud2_interpreter.cpp)
target_compile_options(laserscan_interpreter_node PRIVATE -DNODE)
target_compile_options(LaserScanInterpreterNodelet PRIVATE -DNODELET)
target_compile_options(laserscanarray_interpreter_node PRIVATE -DNODE -DLSARRAY ${OpenMP_FLAGS})
target_compile_options(LaserScanArrayInterpreterNodelet PRIVATE -DNODELET -DLSARRAY ${OpenMP_FLAGS})
target_compile_options(pointcloud2_interpreter_node PRIVATE -DNODE)
target_compile_options(PointCloud2InterpreterNodelet PRIVATE -DNODELET)
target_compile_options(pointcloud2array_interpreter_node PRIVATE -DNODE -DPC2ARRAY ${OpenMP_FLAGS})
target_compile_options(PointCloud2ArrayInterpreterNodelet PRIVATE -DNODELET -DPC2ARRAY ${OpenMP_FLAGS})
add_executable(moving_objects_confidence_enhancer_node src/moving_objects_confidence_enhancer_node.cpp)
add_executable(example_frame_broadcaster_node src/example_frame_broadcaster_node.cpp)
add_executable(example_d435_voxel_echoer_node src/example_d435_voxel_echoer_node.cpp)
add_executable(example_rplidar_echoer_node src/example_rplidar_echoer_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# add_dependencies(laserscan_interpreter_node option find_moving_objects hz_calculator)
# add_dependencies(LaserScanInterpreterNodelet option find_moving_objects hz_calculator)
# add_dependencies(pointcloud2_interpreter_node option find_moving_objects hz_calculator)
# add_dependencies(PointCloud2InterpreterNodelet option find_moving_objects hz_calculator)
# add_dependencies(moving_objects_confidence_enhancer_node ${PROJECT_NAME}_generate_messages)
# add_dependencies(moving_objects_confidence_enhancer_node option)
# add_dependencies(example_frame_broadcaster_node option)
# add_dependencies(example_d435_voxel_echoer_node option)
# add_dependencies(example_rplidar_echoer_node option)
add_dependencies(laserscan_interpreter_node find_moving_objects)
add_dependencies(LaserScanInterpreterNodelet find_moving_objects)
add_dependencies(laserscanarray_interpreter_node find_moving_objects ${catkin_EXPORTED_TARGETS})
add_dependencies(LaserScanArrayInterpreterNodelet find_moving_objects ${catkin_EXPORTED_TARGETS})
add_dependencies(pointcloud2_interpreter_node find_moving_objects)
add_dependencies(PointCloud2InterpreterNodelet find_moving_objects)
add_dependencies(pointcloud2array_interpreter_node find_moving_objects ${catkin_EXPORTED_TARGETS})
add_dependencies(PointCloud2ArrayInterpreterNodelet find_moving_objects ${catkin_EXPORTED_TARGETS})
add_dependencies(moving_objects_confidence_enhancer_node ${PROJECT_NAME}_generate_messages)
# add_dependencies(moving_objects_confidence_enhancer_node option)
# add_dependencies(example_frame_broadcaster_node option)
# add_dependencies(example_d435_voxel_echoer_node)
# add_dependencies(example_rplidar_echoer_node)

## Specify libraries to link a library or executable target against
target_link_libraries(
find_moving_objects
  ${catkin_LIBRARIES}
  m
)

# target_link_libraries(
# hz_calculator
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(
# option
#   ${catkin_LIBRARIES}
# )

target_link_libraries(
laserscan_interpreter_node
  ${catkin_LIBRARIES}
#   option
  find_moving_objects
#   hz_calculator
)

target_link_libraries(
LaserScanInterpreterNodelet
  ${catkin_LIBRARIES}
#   option
  find_moving_objects
#   hz_calculator
)

target_link_libraries(
laserscanarray_interpreter_node
  ${catkin_LIBRARIES}
  ${OpenMP_LIBS}
#   option
  find_moving_objects
#   hz_calculator
)

target_link_libraries(
LaserScanArrayInterpreterNodelet
  ${catkin_LIBRARIES}
  ${OpenMP_LIBS}
#   option
  find_moving_objects
#   hz_calculator
)

target_link_libraries(
pointcloud2_interpreter_node
  ${catkin_LIBRARIES}
#   option
  find_moving_objects
#   hz_calculator
)

target_link_libraries(
PointCloud2InterpreterNodelet
  ${catkin_LIBRARIES}
#   option
  find_moving_objects
#   hz_calculator
)

target_link_libraries(
pointcloud2array_interpreter_node
  ${catkin_LIBRARIES}
  ${OpenMP_LIBS}
#   option
  find_moving_objects
#   hz_calculator
)

target_link_libraries(
PointCloud2ArrayInterpreterNodelet
  ${catkin_LIBRARIES}
  ${OpenMP_LIBS}
#   option
  find_moving_objects
#   hz_calculator
)

target_link_libraries(
moving_objects_confidence_enhancer_node
  ${catkin_LIBRARIES}
#   option
)

target_link_libraries(
example_frame_broadcaster_node
  ${catkin_LIBRARIES}
#   option
)

target_link_libraries(
example_d435_voxel_echoer_node
  ${catkin_LIBRARIES}
)

target_link_libraries(
example_rplidar_echoer_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 FILES_MATCHING PATTERN "*.h"
 PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch rviz logger_config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)


install(TARGETS laserscan_interpreter_node 
                laserscanarray_interpreter_node 
                pointcloud2_interpreter_node
                pointcloud2array_interpreter_node
                LaserScanInterpreterNodelet 
                LaserScanArrayInterpreterNodelet 
                PointCloud2InterpreterNodelet
                PointCloud2ArrayInterpreterNodelet
                moving_objects_confidence_enhancer_node
                example_d435_voxel_echoer_node
                example_rplidar_echoer_node
                example_frame_broadcaster_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_unicorn_sensor_interpreter.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
