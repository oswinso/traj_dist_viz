cmake_minimum_required(VERSION 3.8)
project(traj_dist_viz CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
  nav_msgs
  message_generation
  )
set(CMAKE_AUTOMOC ON)

add_message_files(
  DIRECTORY msg
  FILES
  TrajDist.msg)
generate_messages(DEPENDENCIES
  std_msgs
  nav_msgs
  )

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)

  message("Qt5Widgets_INCLUDE_DIRS: ${Qt5Widgets_INCLUDE_DIRS}")
endif()
add_definitions(-DQT_NO_KEYWORDS)

find_package(Boost REQUIRED COMPONENTS container)

catkin_package()

## ---------------- Eigen ----------------------------
#include(FetchContent)
#set(BUILD_TESTING OFF CACHE INTERNAL "")
#FetchContent_Declare(
#  eigen
#  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
#  GIT_TAG        3.4.0
#  GIT_SHALLOW    TRUE
#)
#FetchContent_MakeAvailable(eigen)

set(SRC_FILES
  src/path_dist_display.cpp
  include/traj_dist_viz/path_dist_display.h
  )

add_library(traj_dist_viz ${SRC_FILES})
target_include_directories(traj_dist_viz PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(traj_dist_viz PRIVATE ${QT_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(traj_dist_viz ${traj_dist_viz_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(test_traj_dist_pub src/test_traj_dist_pub.cpp)
target_include_directories(test_traj_dist_pub PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(test_traj_dist_pub PRIVATE ${catkin_LIBRARIES})
add_dependencies(test_traj_dist_pub ${traj_dist_viz_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
