cmake_minimum_required(VERSION 3.0.0)

project(txsim-examples)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(
  ../inc
  ${CMAKE_BINARY_DIR})

if(WIN32)
  set(LIB_PROTOBUF libprotobuf zlibstatic)
  set(LIB_SYS)
elseif(UNIX)
  set(LIB_PROTOBUF protobuf pthread)
  set(LIB_SYS dl)
endif(WIN32)

find_package(Protobuf REQUIRED)
protobuf_generate_cpp(PROTO_SRC PROTO_HDR
  ../msgs/basic.proto
  ../msgs/location.proto
  ../msgs/trajectory.proto
  ../msgs/traffic.proto
  ../msgs/control.proto
  ../msgs/header.proto
  ../msgs/planStatus.proto
  ../msgs/localLocation.proto
  ../msgs/trajectory.proto
  ../msgs/laneMarks.proto)


# example 1: built into a shared library.
add_library(my-module SHARED
  my_module.cc
  ${PROTO_SRC})
target_link_libraries(my-module
  ${LIB_PROTOBUF})

# example 2: built into a binary executable.
# link_directories(
#   ../lib)
# add_executable(my-module-main
#   my_module_main.cc
#   my_module.cc
#   ${PROTO_SRC})
# target_compile_definitions(my-module-main PRIVATE TXSIM_NO_EXPORTS)
# target_link_libraries(my-module-main
#   txsim-module-service ${LIB_PROTOBUF} ${LIB_SYS})
