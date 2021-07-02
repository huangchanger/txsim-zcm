cmake_minimum_required (VERSION 3.0)
project(txsim-zcm)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

set(SRC ${CMAKE_SOURCE_DIR}/src)

find_package(Protobuf REQUIRED)
protobuf_generate_cpp(PROTO_SRC PROTO_HDR
  ${CMAKE_SOURCE_DIR}/include/proto_msgs/basic.proto
  ${CMAKE_SOURCE_DIR}/include/proto_msgs/header.proto
  ${CMAKE_SOURCE_DIR}/include/proto_msgs/location.proto
  ${CMAKE_SOURCE_DIR}/include/proto_msgs/trajectory.proto
  ${CMAKE_SOURCE_DIR}/include/proto_msgs/planStatus.proto
  ${CMAKE_SOURCE_DIR}/include/proto_msgs/laneMarks.proto
  ${CMAKE_SOURCE_DIR}/include/proto_msgs/traffic.proto
  ${CMAKE_SOURCE_DIR}/include/proto_msgs/control.proto)

# file(GLOB PROTO_SRC ${CMAKE_SOURCE_DIR}/include/proto_msgs/*.cc)
file(GLOB CPPFILES ${SRC}/*.cc)

add_definitions(-DUSE_ZCM)
include_directories("/usr/local/include")


set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${CMAKE_BINARY_DIR})


if(WIN32)
  set(LIB_PROTOBUF libprotobuf zlibstatic)
  set(LIB_SYS)
elseif(UNIX)
  set(LIB_PROTOBUF protobuf pthread)
  set(LIB_SYS dl)
endif(WIN32)



# example 1: built into a shared library.
add_library(my-module SHARED
  ${CPPFILES}
  ${PROTO_SRC})

target_link_libraries(my-module
  ${LIB_PROTOBUF}
  zcm)
