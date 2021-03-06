cmake_minimum_required(VERSION 2.8.3)
project(hand_publisher)

find_package(catkin REQUIRED
            COMPONENTS roscpp tf std_msgs geometry_msgs)
find_package(Boost REQUIRED
            COMPONENTS thread)
find_package(OpenCV REQUIRED)

# use the LSB stuff if possible :)
EXECUTE_PROCESS(
  COMMAND cat /etc/lsb-release
  COMMAND grep DISTRIB_RELEASE
  COMMAND awk -F= "{ print $2 }"
  COMMAND tr "\n" " "
  COMMAND sed "s/ //"
  OUTPUT_VARIABLE LSB_RELEASE
  RESULT_VARIABLE LSB_RELEASE_RESULT
)

if(${LSB_RELEASE} STREQUAL "12.04")
    set(OpenCV_LIBRARIES ${OpenCV_LIBS})
endif(${LSB_RELEASE} STREQUAL "12.04")

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

add_executable(skeleton_to_hand_node ${HEADERS}     src/skeleton_to_hand.cpp
                                                    src/HandPublisher.cpp)
target_link_libraries(skeleton_to_hand_node         ${catkin_LIBRARIES})

add_executable(fabric_vision_node ${HEADERS}        src/fabric_vision_node.cpp
                                                    src/FabricVision.cpp)
target_link_libraries(fabric_vision_node            ${catkin_LIBRARIES}
                                                    ${OpenCV_LIBRARIES})

add_executable(coordination_node ${HEADERS}         src/coordination_node.cpp
                                                    src/Coordination.cpp)
target_link_libraries(coordination_node             ${catkin_LIBRARIES})

#############
## Install ##
#############


#############
## Testing ##
#############

file(GLOB_RECURSE TESTS ${PROJECT_SOURCE_DIR}/test/*.cpp)

foreach(ITEM ${TESTS})
    get_filename_component(filename ${ITEM} NAME_WE)
    catkin_add_gtest(${filename} ${ITEM} src/FabricVision.cpp
                                         src/HandPublisher.cpp
                                         src/SerialCommunication.cpp
                                         src/Coordination.cpp)
    target_link_libraries(${filename} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
endforeach()
