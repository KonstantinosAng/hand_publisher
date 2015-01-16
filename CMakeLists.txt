cmake_minimum_required(VERSION 2.8.3)
project(hand_publisher)

find_package(catkin REQUIRED COMPONENTS roscpp tf std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp tf std_msgs
)

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

file(GLOB_RECURSE HEADERS ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/*.h)

add_executable(${PROJECT_NAME} ${HEADERS}   src/${PROJECT_NAME}.cpp
                                            src/SerialCommunication.cpp
                                            src/HandPublisher.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#############
## Install ##
#############


#############
## Testing ##
#############
