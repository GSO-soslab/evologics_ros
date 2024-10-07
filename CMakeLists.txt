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
find_package(sensor_msgs REQUIRED) # need ?
find_package(nav_msgs REQUIRED) # need ?
find_package(message_filters REQUIRED) # need ?
find_package(geometry_msgs REQUIRED) # need ?
find_package(geographic_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(acomms_msgs REQUIRED)

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

set(ros_lib
  rclcpp 
  std_msgs 
  geometry_msgs
  sensor_msgs
  nav_msgs
  geographic_msgs
  acomms_msgs
  robot_localization
)

set(other_lib
  proto
  goby
  dccl
)

# include_directories(
#   include/seatrac_driver
#   include/evologics_driver
#   include/AT
# )

add_executable(modem_node
  src/node.cpp
  src/modem.cpp
  # include/seatrac_driver/seatrac_driver.cpp
  # include/evologics_driver/evologics_driver.cpp
  # include/AT/ATsentence.cpp
)

## Corrected dependencies for ROS 2 
ament_target_dependencies(modem_node ${ros_lib})

target_link_libraries(modem_node ${other_lib})


####################################################################
# install                      
####################################################################

install(TARGETS modem_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(${ros_lib})
ament_package()