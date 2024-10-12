cmake_minimum_required(VERSION 3.5.1)
project(evologics_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules/")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  acomms_msgs
  std_msgs
  nav_msgs
  message_filters
  geometry_msgs
  geographic_msgs
  tf2_ros
)

find_package(Protobuf REQUIRED)


find_package(goby 3.1 REQUIRED)

find_package(dccl REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED)

include_directories( ${CMAKE_BINARY_DIR} )

add_subdirectory(evologics_driver)


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
  # INCLUDE_DIRS 
  # LIBRARIES
  CATKIN_DEPENDS roscpp acomms_msgs std_msgs geometry_msgs geographic_msgs 
  # DEPENDS
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(modem_node
  src/modem.cpp
)

add_dependencies(modem_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(modem_node
  ${catkin_LIBRARIES}
  ${PROTOBUF_LIBRARY}
  goby
  dccl
  evologics_driver
)