cmake_minimum_required(VERSION 2.8.3)
project(hand_publisher)

find_package(catkin REQUIRED COMPONENTS roscpp tf std_msgs)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tf std_msgs
)

set(PACKAGE_PATH "${PROJECT_SOURCE_DIR}")
configure_file(${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/config.h.in ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/config.h)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS}
                            ${OpenCV_INCLUDE_DIR}
                            ${Boost_INCLUDE_DIRS})

file(GLOB_RECURSE HEADERS ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h)

add_executable(serial_communication_node ${HEADERS} src/serial_communication_node.cpp
                                                    src/SerialCommunication.cpp)
target_link_libraries(serial_communication_node     ${catkin_LIBRARIES}
                                                    ${Boost_LIBRARIES})

add_executable(tf_to_string_node ${HEADERS}         src/tf_to_string_node.cpp
                                                    src/HandPublisher.cpp)
target_link_libraries(tf_to_string_node             ${catkin_LIBRARIES})

add_executable(fabric_vision_node ${HEADERS}        src/fabric_vision_node.cpp
                                                    src/FabricVision.cpp)
target_link_libraries(fabric_vision_node            ${catkin_LIBRARIES}
                                                    ${OpenCV_LIBRARIES})

#############
## Install ##
#############


#############
## Testing ##
#############
