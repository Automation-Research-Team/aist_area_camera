cmake_minimum_required(VERSION 2.8.12)
project(aist_area_camera)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/local/share/cmake/TU")

find_package(TULibs REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    roslib
    image_transport
    sensor_msgs
    ddynamic_reconfigure
    pluginlib
    nodelet
)

include_directories(include
  ${PROJECT_SOURCE_DIR}/../include
  ${catkin_INCLUDE_DIRS}
  ${TULIBS_INCLUDE_DIR}
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    image_transport
)

add_compile_options(-w)
add_compile_options(-std=c++17)
add_compile_options(-fpermissive)
add_compile_options(-pthread)
add_compile_options(-O2)

add_executable(
  v4l2_camera
  src/v4l2_camera.cpp
  src/V4L2CameraArrayNode.cpp
)

add_dependencies(
  v4l2_camera
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  v4l2_camera
  debug ${catkin_LIBRARIES}
  optimized ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TUV4L2_LIB}
  ${TUTOOLS_LIB}
)

add_library(
  v4l2_camera_nodelet
  src/v4l2_nodelet.cpp
  src/V4L2CameraArrayNode.cpp
)

add_dependencies(
  v4l2_camera_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  v4l2_camera_nodelet
  debug ${catkin_LIBRARIES}
  optimized ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TUV4L2_LIB}
  ${TUTOOLS_LIB}
)

add_executable(
  iidc_camera
  src/iidc_camera.cpp
  src/IIDCCameraArrayNode.cpp
)

add_dependencies(
  iidc_camera
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  iidc_camera
  debug ${catkin_LIBRARIES}
  optimized ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TUIIDC_LIB}
  ${TUTOOLS_LIB}
)

add_library(
  iidc_camera_nodelet
  src/iidc_nodelet.cpp
  src/IIDCCameraArrayNode.cpp
)

add_dependencies(
  iidc_camera_nodelet
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  iidc_camera_nodelet
  debug ${catkin_LIBRARIES}
  optimized ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${TUIIDC_LIB}
  ${TUTOOLS_LIB}
)

install(
  TARGETS
    v4l2_camera v4l2_camera_nodelet iidc_camera iidc_camera_nodelet
  ARCHIVE
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
