cmake_minimum_required(VERSION 3.5.1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

project(evologics_ros)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules/")

## Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(acomms_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Protobuf REQUIRED)
find_package(goby 3.1 REQUIRED)
find_package(dccl REQUIRED) # need ?

## Build the protobuf messages
file(GLOB ProtoFiles "${CMAKE_CURRENT_SOURCE_DIR}/src/proto/*.proto")
PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${ProtoFiles})
add_library(proto STATIC ${ProtoSources} ${ProtoHeaders})
target_link_libraries(proto ${PROTOBUF_LIBRARY})

include_directories(${CMAKE_BINARY_DIR})

####################################################################
# build                      
####################################################################

add_subdirectory(evologics_driver)

set(ros_lib
  rclcpp 
  std_msgs 
  sensor_msgs
  geometry_msgs
  geographic_msgs
  acomms_msgs
  robot_localization
  tf2
  tf2_geometry_msgs
)

set(other_lib
  proto
  goby
  dccl
  evologics_driver
)

add_executable(${PROJECT_NAME}_node
  src/node.cpp
  src/modem.cpp
)

## Corrected dependencies for ROS 2 
ament_target_dependencies(${PROJECT_NAME}_node ${ros_lib})
target_include_directories(${PROJECT_NAME}_node PRIVATE src evologics_driver/src)
target_link_libraries(${PROJECT_NAME}_node ${other_lib})


####################################################################
# install                      
####################################################################

install(TARGETS ${PROJECT_NAME}_node evologics_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY include/
#   DESTINATION include/
# )

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${ros_lib})
ament_export_libraries(evologics_driver)
ament_package()