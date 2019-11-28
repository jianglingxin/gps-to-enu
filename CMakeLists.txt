cmake_minimum_required(VERSION 2.8.3)
project(gps-to-enu)


set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS on)


include_directories(${PROJECT_BINARY_DIR})

find_package(Eigen3 REQUIRED)
if (EIGEN_FOUND)
  if (EIGEN_VERSION VERSION_LESS 3.3.1)
    message(FATAL_ERROR "-- Sophus(3rdparty) requires Eigen version >= 3.3.1, "
        "detected version of Eigen is: ${EIGEN_VERSION}")
  endif (EIGEN_VERSION VERSION_LESS 3.3.1)
endif (EIGEN_FOUND)
include_directories(${EIGEN3_INCLUDE_DIR})

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gps-to-enu
  CATKIN_DEPENDS roscpp
)


find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARIY_DIRS})
add_definitions(${PCL_DEFINITIONS})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/gps_init.cc
    )
set_target_properties(${PROJECT_NAME} PROPERTIES LINKER_LANGUAGE CXX)
target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
    ${catkin_LIBRARIES}
    )

add_executable(gps_to_odom ros_node/gps_to_odom.cpp)
target_link_libraries(gps_to_odom
    ${PROJECT_NAME}
    )


#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME} gps_to_odom
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
install(DIRECTORY param
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_unity-build-car.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
