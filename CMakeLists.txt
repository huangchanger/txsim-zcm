cmake_minimum_required (VERSION 3.0)
project(txsim-zcm)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SRC ${CMAKE_SOURCE_DIR}/src)
set(THIRD_PARTY ${CMAKE_SOURCE_DIR}/3rd)

# path
include_directories(${SRC})
include_directories(${THIRD_PARTY})

add_definitions(-DUSE_ZCM)
include_directories(${THIRD_PARTY}/msg/zcm_cpp)
include_directories("/usr/local/include")



set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_BINARY_DIR}
  ${THIRD_PARTY}/msg)

if(WIN32)
  set(LIB_PROTOBUF libprotobuf zlibstatic)
  set(LIB_SYS)
elseif(UNIX)
  set(LIB_PROTOBUF protobuf pthread)
  set(LIB_SYS dl)
endif(WIN32)

find_package(Protobuf REQUIRED)
protobuf_generate_cpp(PROTO_SRC PROTO_HDR
  ${THIRD_PARTY}/msg/proto_msgs/basic.proto
  ${THIRD_PARTY}/msg/proto_msgs/location.proto
  ${THIRD_PARTY}/msg/proto_msgs/trajectory.proto
  ${THIRD_PARTY}/msg/proto_msgs/traffic.proto)

# example 1: built into a shared library.
add_library(my-module SHARED
  ${SRC}/txsim/my_module.cc
  ${CMAKE_SOURCE_DIR})
target_link_libraries(my-module
  ${LIB_PROTOBUF})
