cmake_minimum_required(VERSION 2.8.12)
project(v4l2_camera)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/local/share/cmake/TU")

find_package(TULibs REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    roslib
    std_msgs
    image_transport
    sensor_msgs
    dynamic_reconfigure
    tf
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${TULIBS_INCLUDE_DIR}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    message_runtime
    std_msgs
    sensor_msgs
    image_transport
)

add_compile_options(-w)
add_compile_options(-std=c++14)
add_compile_options(-fpermissive)
add_compile_options(-pthread)

add_executable(
  ${PROJECT_NAME}
  src/main.cpp
)

add_dependencies(
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  ${PROJECT_NAME}
  debug ${catkin_LIBRARIES}
  optimized ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TUV4L2PP_LIB}
)

install(
  TARGETS
    ${PROJECT_NAME}
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
