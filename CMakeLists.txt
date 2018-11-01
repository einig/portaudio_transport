cmake_minimum_required(VERSION 2.8.3)
project(portaudio_transport)

add_compile_options(-std=c++11)

set(HEADERS
    include/${PROJECT_NAME}/recording_publisher.h
    include/${PROJECT_NAME}/playback_subscriber.h
)

set(SOURCE_FILES
    src/recording_publisher.cpp
    src/playback_subscriber.cpp
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

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS roscpp message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} ${SOURCE_FILES} ${HEADERS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_publisher src/portaudio_transport_publisher.cpp)
add_dependencies(${PROJECT_NAME}_publisher ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_publisher
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  sndfile
  portaudiocpp
  portaudio
)

add_executable(${PROJECT_NAME}_subscriber src/portaudio_transport_subscriber.cpp)
add_dependencies(${PROJECT_NAME}_subscriber ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_subscriber
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  sndfile
  portaudiocpp
  portaudio
)