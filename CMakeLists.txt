cmake_minimum_required(VERSION 3.5)
project(stage_ros)



# C / C++ versions
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

# Compiler flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



## SYTEM DEPENDENCIES (found with CMake's conventions)
# Stage Simulator
if (DEFINED STAGE_PACKAGE_PATH)
  message(STATUS "STAGE_PACKAGE_PATH CMake variable is set: ${STAGE_PACKAGE_PATH}")
  find_package(stage REQUIRED PATHS ${STAGE_PACKAGE_PATH} NO_DEFAULT_PATH)
else()
  find_package(stage QUIET)
  if (NOT stage_FOUND)
    message(STATUS "stage not found in CMake default modules path. "
                  "Trying to find in default locations...")
    list(APPEND CMAKE_PREFIX_PATH "/opt/stage")
    message(STATUS "stage ROS-independent default location: /opt/stage")
    message(STATUS "CMake modules path: ${CMAKE_MODULE_PATH}")
    find_package(stage REQUIRED)
  endif()
endif()

message(STATUS "stage found at: ${stage_DIR}")





##########################################
##### ROBOT OPERATING SYSTEM (ROS) 1 #####
##########################################

if("$ENV{ROS_VERSION}" STREQUAL "1")

message(STATUS "Robot Operating System (ROS) detected: ROS $ENV{ROS_VERSION} $ENV{ROS_DISTRO}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2_geometry_msgs
    tf2_ros
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  # INCLUDE_DIRS
  #   include
  # LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    roscpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2_geometry_msgs
    tf2_ros
  DEPENDS
    stage
)

link_directories(${catkin_LIBRARY_DIRS})

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${STAGE_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(stageros
  src/stageros.cpp
)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(stageros
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

set(${PROJECT_NAME}_extra_libs "")
if(UNIX AND NOT APPLE)
  set(${PROJECT_NAME}_extra_libs dl)
endif()

## Specify libraries to link a library or executable target against
target_link_libraries(stageros
  ${catkin_LIBRARIES}
  ${STAGE_LIBRARIES}
  ${${PROJECT_NAME}_extra_libs}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(PROGRAMS scripts/upgrade-world.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
install(TARGETS stageros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING
    PATTERN "*.launch"
)





##########################################
##### ROBOT OPERATING SYSTEM (ROS) 2 #####
##########################################

elseif("$ENV{ROS_VERSION}" STREQUAL "2")

message(STATUS "Robot Operating System (ROS) detected: ROS $ENV{ROS_VERSION} $ENV{ROS_DISTRO}")

## Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

## Declare a C++ executable
add_executable(stageros2 src/stageros2.cpp)

set(${PROJECT_NAME}_extra_libs "")
if(UNIX AND NOT APPLE)
  set(${PROJECT_NAME}_extra_libs dl)
endif()

## C++ executable specific include directories
target_include_directories(stageros2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${STAGE_INCLUDE_DIRS}
)

## C++ executable specific target dependencies
target_link_libraries(stageros2
  ${STAGE_LIBRARIES}
  ${${PROJECT_NAME}_extra_libs}
)

## C++ executable ament specific target dependencies
ament_target_dependencies(stageros2
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
  tf2_ros
)

#############
## Install ##
#############

install(TARGETS stageros2
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/upgrade-world.sh
  DESTINATION bin/${PROJECT_NAME}
)

install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING
    PATTERN "*.launch.xml"
    PATTERN "*.launch.py"
)

## Testing

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()





## !!!NO ROBOT OPERATING SYSTEM VERSION FOUND!!!

else()

message(FATAL_ERROR "ROS_VERSION environment variable not set...")

endif()
