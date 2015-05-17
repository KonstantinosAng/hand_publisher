cmake_minimum_required(VERSION 2.8.3)
project(hand_publisher)

find_package(catkin REQUIRED
            COMPONENTS roscpp tf std_msgs geometry_msgs k2_client)
find_package(Boost REQUIRED
            COMPONENTS thread)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tf std_msgs k2_client
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
                                                    src/HandPublisher.cpp
                                                    src/KinectVision.cpp)
target_link_libraries(skeleton_to_hand_node         ${catkin_LIBRARIES})
add_dependencies(skeleton_to_hand_node              k2_client_gencpp)

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

catkin_add_gtest(test_coordination          test/test_coordination.cpp
                                            src/Coordination.cpp)
target_link_libraries(test_coordination     ${catkin_LIBRARIES})

catkin_add_gtest(test_hand                  test/test_hand.cpp
                                            src/HandPublisher.cpp)
target_link_libraries(test_hand             ${catkin_LIBRARIES})

catkin_add_gtest(test_kinect                test/test_kinect.cpp
                                            src/KinectVision.cpp)
target_link_libraries(test_kinect           ${catkin_LIBRARIES})

catkin_add_gtest(test_vision                test/test_vision.cpp
                                            src/FabricVision.cpp)
target_link_libraries(test_vision           ${catkin_LIBRARIES})
