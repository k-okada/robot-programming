cmake_minimum_required(VERSION 2.8.3)
project(turtleboteus)

find_package(catkin REQUIRED kobuki_msgs control_msgs move_base_msgs roseus rostest) # add roseus to gen messages

catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(pr2eus QUIET)
  if(pr2eus_FOUND)
    add_rostest(test/test-turtlebot.test)
  endif()
endif()

##
add_definitions(-Wall)

include_directories(/usr/include /usr/X11R6/include ${euslisp_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_library(roseus_aux src/roseus_aux.cpp)
set_target_properties(roseus_aux PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/euslisp)
set_target_properties(roseus_aux PROPERTIES PREFIX "" SUFFIX ".so")

# compile flags
add_definitions(-O2 -DNDEBUG -Wno-write-strings -Wno-unused-but-set-variable -Wno-comment)
add_definitions(-Di486 -DLinux -D_REENTRANT -DVERSION='\"9.00\"' -DTHREADED -DPTHREAD -DX11R6_1)
add_definitions('-DREPOVERSION="\\"${REPOVERSION}\\""')
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "amd64(.*)" OR
   "${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64(.*)" OR
   "${CMAKE_SYSTEM_PROCESSOR}" MATCHES "aarch64(.*)" )
 add_definitions(-Dx86_64)
else()
 add_definitions(-Di486)
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_SHARED_LIBRARY_CREATE_C_FLAGS "${CMAKE_SHARED_LIBRARY_CREATE_C_FLAGS} -undefined dynamic_lookup")
  set(CMAKE_SHARED_LIBRARY_CREATE_CXX_FLAGS "${CMAKE_SHARED_LIBRARY_CREATE_CXX_FLAGS} -undefined dynamic_lookup")
endif()
