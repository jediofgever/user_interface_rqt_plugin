cmake_minimum_required(VERSION 2.8.3)
project(user_interface_rqt_plugin)

add_compile_options(-std=c++11)
include(FindPkgConfig)
pkg_check_modules(CURLPP REQUIRED curlpp)
find_package(jsoncpp CONFIG REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
  moveit_core
  moveit_visual_tools
  moveit_ros_planning_interface
  arm_perception_utilities
)
find_package(Qt5Widgets REQUIRED)
 


find_package(Eigen3 REQUIRED)
# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

 

include_directories(
  INCLUDE_DIRS  
  include
  ${catkin_INCLUDE_DIRS}
  ${arm_perception_utilities_INCLUDE_DIRS}
)

set(user_interface_rqt_plugin_HDRS
  include/user_interface_rqt_plugin/popup.h
  include/user_interface_rqt_plugin/robot_IO.h
  include/user_interface_rqt_plugin/rqt_gui_control.h
)
set(user_interface_rqt_plugin_SRCS
  src/popup.cpp
  src/robot_IO.cpp
  src/rqt_gui_control.cpp
  src/RobotController.cpp

)
set(user_interface_rqt_plugin_UIS
  resources/real_ui.ui
)

catkin_package(
  INCLUDE_DIRS ${user_interface_rqt_plugin_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    rqt_gui 
    rqt_gui_cpp    
    moveit_visual_tools 
    moveit_core
    moveit_ros_planning_interface
    arm_perception_utilities
  DEPENDS
    EIGEN3
)
catkin_python_setup()
qt5_wrap_cpp(user_interface_rqt_plugin_MOCS ${user_interface_rqt_plugin_HDRS})
qt5_wrap_ui(user_interface_rqt_plugin_UIS_H ${user_interface_rqt_plugin_UIS})

link_directories(${catkin_LIBRARY_DIRS})

include_directories(SYSTEM ${THIS_PACKAGE_INCLUDE_DIRS} ${user_interface_rqt_plugin_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} )
add_library(${PROJECT_NAME} ${user_interface_rqt_plugin_SRCS} ${user_interface_rqt_plugin_MOCS} ${user_interface_rqt_plugin_UIS_H})
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} Qt5::Widgets  ${CURLPP_LDFLAGS} jsoncpp_lib)
install(TARGETS ${PROJECT_NAME} DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})



find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/user_interface_rqt_plugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

