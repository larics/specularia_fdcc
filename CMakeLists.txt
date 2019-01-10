cmake_minimum_required(VERSION 2.8.3)
project(fdcc)

find_package(Boost REQUIRED 
          system
          filesystem 
          date_time 
          thread)

find_package(catkin REQUIRED COMPONENTS 
          roscpp
          rospy
          roslib
          cmake_modules
          geometry_msgs
          sensor_msgs
          std_msgs
          message_generation
)

add_message_files(
  FILES
  fdcc_state.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    ##actionlib_msgs
    #geometry_msgs
    #std_msgs
)
# We need to add the project source path to the CMake module path so that
# the FindRBDL.cmake script can be found.
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR} )



# RBDL
FIND_PACKAGE (RBDL COMPONENTS URDFReader REQUIRED)




catkin_package(
    INCLUDE_DIRS src include 
    LIBRARIES  ${Boost_LIBRARIES} ${RBDL_LIBRARY} yaml-cpp
    CATKIN_DEPENDS  message_runtime geometry_msgs roscpp rospy std_msgs 
    DEPENDS system_lib boost Eigen RBDL
)

# Eigen
find_package(Eigen REQUIRED)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS} ${RBDL_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Add library
add_library(FDCC src/fdcc.cpp)
target_link_libraries(FDCC ${catkin_LIBRARIES} ${RBDL_LIBRARY} ${RBDL_URDFReader_LIBRARY})

# Add library
add_library(PID src/pid_controller.cpp)
target_link_libraries(PID ${catkin_LIBRARIES})

# Add executable
add_executable(FDCC_node src/main.cpp)
target_link_libraries(FDCC_node FDCC PID ${catkin_LIBRARIES} ${RBDL_LIBRARY} ${RBDL_URDFReader_LIBRARY} yaml-cpp)
add_dependencies(FDCC_node FDCC PID ${catkin_EXPORTED_TARGETS})

add_executable(RBDL_example src/example.cpp)
target_link_libraries(RBDL_example  ${catkin_LIBRARIES} ${RBDL_LIBRARY})
