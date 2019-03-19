cmake_minimum_required(VERSION 2.8.3)
project(inertial_sense)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)
find_package(Threads)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX" )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX")

add_message_files(
  FILES
  GTime.msg
  SatInfo.msg
  GPS.msg
  GPSInfo.msg
  PreIntIMU.msg
  RTKInfo.msg
  RTKRel.msg
  GlonassEphemeris.msg
  GNSSEphemeris.msg
  GNSSObservation.msg
  GNSSObsVec.msg
)

add_service_files(
  FILES
  FirmwareUpdate.srv
  refLLAUpdate.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ../../../cpp/SDK/src #Reference current version of SDK instead of git submodule
)

add_subdirectory(../../../cpp/SDK lib/InertialSenseSDK) #Reference current version of SDK instead of git submodule

add_executable(inertial_sense_node
        src/inertial_sense.cpp
        include/inertial_sense.h
)
target_link_libraries(inertial_sense_node InertialSense ${catkin_LIBRARIES})
add_dependencies(inertial_sense_node inertial_sense_generate_messages_cpp)
