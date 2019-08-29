cmake_minimum_required(VERSION 2.8.3)
project(lidar_motion)

add_definitions(-std=c++11)
find_package(autoware_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  geometry_msgs
  tf
  autoware_msgs
  )


set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  CATKIN_DEPENDS
  roscpp
  pcl_ros
  autoware_msgs
  tf
  )

include_directories(
  ${autoware_msgs_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  include)

link_directories(${PCL_LIBRARY_DIRS})

#imm_ukf_pda
# imm_ukf_pda_track
add_executable(lidar_motion
        src/imm_ukf_pda_main.cpp
        src/imm_ukf_pda.cpp
        src/ukf.cpp
        )
target_link_libraries(lidar_motion
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )
add_dependencies(lidar_motion
        ${catkin_EXPORTED_TARGETS}
        )


install(TARGETS
        lidar_motion
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )
