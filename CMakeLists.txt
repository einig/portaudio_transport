cmake_minimum_required(VERSION 2.8.3)
project(portaudio_transport)

add_compile_options(-std=c++11)

set(HEADERS
    include/${PROJECT_NAME}/recording_publisher.h
)

set(SOURCE_FILES
    src/recording_publisher.cpp
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  AudioChannel.msg
  AudioTransport.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

find_package(sndfile)
find_package(portaudiocpp)
find_package(portaudio)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib ${SOURCE_FILES} ${HEADERS})
add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_publisher src/portaudio_transport_publisher.cpp)
add_dependencies(${PROJECT_NAME}_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_publisher
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
  sndfile
  portaudiocpp
  portaudio
)