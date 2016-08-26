cmake_minimum_required(VERSION 2.8.3)
list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

# PACKAGE SETUP ########################################

project(status_panel)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rviz std_msgs
  DEPENDS yaml-cpp
)

find_package(catkin REQUIRED COMPONENTS std_msgs roscpp)
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

# QT ########################################

include(${QT_USE_FILE})
add_definitions(-DQT_NO_KEYWORDS)


# FILES ########################################

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/
)
FILE(GLOB HFILES include/${PROJECT_NAME}/*.h)
FILE(GLOB CPPFILES src/*.cpp)
qt4_wrap_cpp(MOC_FILES
  ${HFILES}
)
set(SOURCE_FILES
  ${CPPFILES}
  ${MOC_FILES}
)

# LIBRARY ########################################
add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

# INSTALL ########################################

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)
