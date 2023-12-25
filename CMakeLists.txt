cmake_minimum_required(VERSION 3.0.2)
project(kortex_hardware)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    controller_manager
    hardware_interface
    sensor_msgs
    message_generation
    )

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
link_directories(${PROJECT_SOURCE_DIR}/lib/release)
link_directories()

#############
## Msg/Srv ##
#############
add_service_files(
  FILES
  ModeService.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
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
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    std_msgs
    roscpp
    controller_manager
    hardware_interface
    sensor_msgs
)
##################
## Clang-format ##
##################
include(ClangFormat.cmake)
file(GLOB_RECURSE FILES
    ${PROJECT_SOURCE_DIR}/src/*.[ch]pp
)
file(GLOB FILES_INCLUDE 
    ${PROJECT_SOURCE_DIR}/include/*.h
)
set(FILES ${FILES} ${FILES_INCLUDE})


clang_format_setup(VERSION 10)

if (CLANG_FORMAT_EXECUTABLE)
  clang_format_add_sources(${FILES})

  clang_format_add_targets()
endif()

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${libkindrv_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/client
  ${PROJECT_SOURCE_DIR}/include/client_stubs
  ${PROJECT_SOURCE_DIR}/include/messages
  ${PROJECT_SOURCE_DIR}/include/common
)

add_executable(kortex_hardware src/main.cpp src/Gen3Robot.cpp src/LowPassFilter.cpp)

set(KINOVA_ARM_LIBS ${catkin_LIBRARIES})
set(KINOVA_ARM_LIBS
    ${KINOVA_ARM_LIBS}
    -l:libKortexApiCpp_without_protobuf.a
    -l:libKortexApiCpp.a)

## Specify libraries to link a library or executable target against
target_link_libraries(kortex_hardware
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${pinocchio_LIBRARIES}
  ${KINOVA_ARM_LIBS}
)
