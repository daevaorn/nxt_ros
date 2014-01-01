cmake_minimum_required(VERSION 2.8.3)
project(nxt_rviz_plugin)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

set(SOURCE_FILES src/nxt_ultrasonic_display.cpp
                 src/nxt_color_display.cpp 
                 src/init.cpp)


include(CMakeDetermineSystem)
if(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  add_library(${PROJECT_NAME} MODULE ${SOURCE_FILES})
else(CMAKE_SYSTEM_NAME MATCHES "Darwin")
  add_library(${PROJECT_NAME} ${SOURCE_FILES})
endif(CMAKE_SYSTEM_NAME MATCHES "Darwin")

find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
include_directories(${QT_INCLUDE_DIRS})
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

pkg_check_modules(OGRE OGRE)
include_directories(${OGRE_INCLUDE_DIRS})
link_directories(${OGRE_LIBRARY_DIRS})

target_link_libraries(${PROJECT_NAME} ${wxWidgets_LIBRARIES} ${OGRE_LIBRARIES})

find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES}) 

qt4_wrap_cpp(MOC_FILES
  src/nxt_ultrasonic_display.h
  src/nxt_color_display.h
)

set(SOURCE_FILES
  src/nxt_ultrasonic_display.cpp
  src/xt_color_display.cpp
  ${MOC_FILES}
)

catkin_package()
