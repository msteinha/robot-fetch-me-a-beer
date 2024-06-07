cmake_minimum_required(VERSION 3.0.2)
project(arl_manipulation_baseline)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
)
catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs
  
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
file(GLOB SCRIPTS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} scripts/*)
foreach(SCRIPT IN LISTS SCRIPTS)
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${SCRIPT})
        list(REMOVE_ITEM SCRIPTS ${SCRIPT})
    endif()
endforeach()

catkin_install_python(PROGRAMS ${SCRIPTS}
  scripts/evaluation.py
  src/arl_manipulation_baseline/smoothness.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)