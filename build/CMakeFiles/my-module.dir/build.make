# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/yxj/Projects/txsim-zcm

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yxj/Projects/txsim-zcm/build

# Include any dependencies generated for this target.
include CMakeFiles/my-module.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/my-module.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/my-module.dir/flags.make

basic.pb.cc: ../include/proto_msgs/basic.proto
basic.pb.cc: /usr/local/bin/protoc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/yxj/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Running C++ protocol buffer compiler on /home/yxj/Projects/txsim-zcm/include/proto_msgs/basic.proto"
	/usr/local/bin/protoc --cpp_out=/home/yxj/Projects/txsim-zcm/build -I /home/yxj/Projects/txsim-zcm/include/proto_msgs /home/yxj/Projects/txsim-zcm/include/proto_msgs/basic.proto

basic.pb.h: basic.pb.cc
	@$(CMAKE_COMMAND) -E touch_nocreate basic.pb.h

header.pb.cc: ../include/proto_msgs/header.proto
header.pb.cc: /usr/local/bin/protoc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/yxj/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Running C++ protocol buffer compiler on /home/yxj/Projects/txsim-zcm/include/proto_msgs/header.proto"
	/usr/local/bin/protoc --cpp_out=/home/yxj/Projects/txsim-zcm/build -I /home/yxj/Projects/txsim-zcm/include/proto_msgs /home/yxj/Projects/txsim-zcm/include/proto_msgs/header.proto

header.pb.h: header.pb.cc
	@$(CMAKE_COMMAND) -E touch_nocreate header.pb.h

location.pb.cc: ../include/proto_msgs/location.proto
location.pb.cc: /usr/local/bin/protoc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/yxj/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Running C++ protocol buffer compiler on /home/yxj/Projects/txsim-zcm/include/proto_msgs/location.proto"
	/usr/local/bin/protoc --cpp_out=/home/yxj/Projects/txsim-zcm/build -I /home/yxj/Projects/txsim-zcm/include/proto_msgs /home/yxj/Projects/txsim-zcm/include/proto_msgs/location.proto

location.pb.h: location.pb.cc
	@$(CMAKE_COMMAND) -E touch_nocreate location.pb.h

trajectory.pb.cc: ../include/proto_msgs/trajectory.proto
trajectory.pb.cc: /usr/local/bin/protoc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/yxj/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Running C++ protocol buffer compiler on /home/yxj/Projects/txsim-zcm/include/proto_msgs/trajectory.proto"
	/usr/local/bin/protoc --cpp_out=/home/yxj/Projects/txsim-zcm/build -I /home/yxj/Projects/txsim-zcm/include/proto_msgs /home/yxj/Projects/txsim-zcm/include/proto_msgs/trajectory.proto

trajectory.pb.h: trajectory.pb.cc
	@$(CMAKE_COMMAND) -E touch_nocreate trajectory.pb.h

planStatus.pb.cc: ../include/proto_msgs/planStatus.proto
planStatus.pb.cc: /usr/local/bin/protoc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/yxj/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Running C++ protocol buffer compiler on /home/yxj/Projects/txsim-zcm/include/proto_msgs/planStatus.proto"
	/usr/local/bin/protoc --cpp_out=/home/yxj/Projects/txsim-zcm/build -I /home/yxj/Projects/txsim-zcm/include/proto_msgs /home/yxj/Projects/txsim-zcm/include/proto_msgs/planStatus.proto

planStatus.pb.h: planStatus.pb.cc
	@$(CMAKE_COMMAND) -E touch_nocreate planStatus.pb.h

laneMarks.pb.cc: ../include/proto_msgs/laneMarks.proto
laneMarks.pb.cc: /usr/local/bin/protoc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/yxj/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Running C++ protocol buffer compiler on /home/yxj/Projects/txsim-zcm/include/proto_msgs/laneMarks.proto"
	/usr/local/bin/protoc --cpp_out=/home/yxj/Projects/txsim-zcm/build -I /home/yxj/Projects/txsim-zcm/include/proto_msgs /home/yxj/Projects/txsim-zcm/include/proto_msgs/laneMarks.proto

laneMarks.pb.h: laneMarks.pb.cc
	@$(CMAKE_COMMAND) -E touch_nocreate laneMarks.pb.h

traffic.pb.cc: ../include/proto_msgs/traffic.proto
traffic.pb.cc: /usr/local/bin/protoc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/yxj/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Running C++ protocol buffer compiler on /home/yxj/Projects/txsim-zcm/include/proto_msgs/traffic.proto"
	/usr/local/bin/protoc --cpp_out=/home/yxj/Projects/txsim-zcm/build -I /home/yxj/Projects/txsim-zcm/include/proto_msgs /home/yxj/Projects/txsim-zcm/include/proto_msgs/traffic.proto

traffic.pb.h: traffic.pb.cc
	@$(CMAKE_COMMAND) -E touch_nocreate traffic.pb.h

control.pb.cc: ../include/proto_msgs/control.proto
control.pb.cc: /usr/local/bin/protoc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/yxj/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Running C++ protocol buffer compiler on /home/yxj/Projects/txsim-zcm/include/proto_msgs/control.proto"
	/usr/local/bin/protoc --cpp_out=/home/yxj/Projects/txsim-zcm/build -I /home/yxj/Projects/txsim-zcm/include/proto_msgs /home/yxj/Projects/txsim-zcm/include/proto_msgs/control.proto

control.pb.h: control.pb.cc
	@$(CMAKE_COMMAND) -E touch_nocreate control.pb.h

CMakeFiles/my-module.dir/src/MessageManager.cc.o: CMakeFiles/my-module.dir/flags.make
CMakeFiles/my-module.dir/src/MessageManager.cc.o: ../src/MessageManager.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yxj/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/my-module.dir/src/MessageManager.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/my-module.dir/src/MessageManager.cc.o -c /home/yxj/Projects/txsim-zcm/src/MessageManager.cc

CMakeFiles/my-module.dir/src/MessageManager.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my-module.dir/src/MessageManager.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yxj/Projects/txsim-zcm/src/MessageManager.cc > CMakeFiles/my-module.dir/src/MessageManager.cc.i

CMakeFiles/my-module.dir/src/MessageManager.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my-module.dir/src/MessageManager.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yxj/Projects/txsim-zcm/src/MessageManager.cc -o CMakeFiles/my-module.dir/src/MessageManager.cc.s

CMakeFiles/my-module.dir/src/MessageManager.cc.o.requires:

.PHONY : CMakeFiles/my-module.dir/src/MessageManager.cc.o.requires

CMakeFiles/my-module.dir/src/MessageManager.cc.o.provides: CMakeFiles/my-module.dir/src/MessageManager.cc.o.requires
	$(MAKE) -f CMakeFiles/my-module.dir/build.make CMakeFiles/my-module.dir/src/MessageManager.cc.o.provides.build
.PHONY : CMakeFiles/my-module.dir/src/MessageManager.cc.o.provides

CMakeFiles/my-module.dir/src/MessageManager.cc.o.provides.build: CMakeFiles/my-module.dir/src/MessageManager.cc.o


CMakeFiles/my-module.dir/src/MessageManagerBase.cc.o: CMakeFiles/my-module.dir/flags.make
CMakeFiles/my-module.dir/src/MessageManagerBase.cc.o: ../src/MessageManagerBase.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yxj/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/my-module.dir/src/MessageManagerBase.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/my-module.dir/src/MessageManagerBase.cc.o -c /home/yxj/Projects/txsim-zcm/src/MessageManagerBase.cc

CMakeFiles/my-module.dir/src/MessageManagerBase.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my-module.dir/src/MessageManagerBase.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yxj/Projects/txsim-zcm/src/MessageManagerBase.cc > CMakeFiles/my-module.dir/src/MessageManagerBase.cc.i

CMakeFiles/my-module.dir/src/MessageManagerBase.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my-module.dir/src/MessageManagerBase.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yxj/Projects/txsim-zcm/src/MessageManagerBase.cc -o CMakeFiles/my-module.dir/src/MessageManagerBase.cc.s

CMakeFiles/my-module.dir/src/MessageManagerBase.cc.o.requires:

.PHONY : CMakeFiles/my-module.dir/src/MessageManagerBase.cc.o.requires

CMakeFiles/my-module.dir/src/MessageManagerBase.cc.o.provides: CMakeFiles/my-module.dir/src/MessageManagerBase.cc.o.requires
	$(MAKE) -f CMakeFiles/my-module.dir/build.make CMakeFiles/my-module.dir/src/MessageManagerBase.cc.o.provides.build
.PHONY : CMakeFiles/my-module.dir/src/MessageManagerBase.cc.o.provides

CMakeFiles/my-module.dir/src/MessageManagerBase.cc.o.provides.build: CMakeFiles/my-module.dir/src/MessageManagerBase.cc.o


CMakeFiles/my-module.dir/src/WGS84UTM.cc.o: CMakeFiles/my-module.dir/flags.make
CMakeFiles/my-module.dir/src/WGS84UTM.cc.o: ../src/WGS84UTM.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yxj/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/my-module.dir/src/WGS84UTM.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/my-module.dir/src/WGS84UTM.cc.o -c /home/yxj/Projects/txsim-zcm/src/WGS84UTM.cc

CMakeFiles/my-module.dir/src/WGS84UTM.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my-module.dir/src/WGS84UTM.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yxj/Projects/txsim-zcm/src/WGS84UTM.cc > CMakeFiles/my-module.dir/src/WGS84UTM.cc.i

CMakeFiles/my-module.dir/src/WGS84UTM.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my-module.dir/src/WGS84UTM.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yxj/Projects/txsim-zcm/src/WGS84UTM.cc -o CMakeFiles/my-module.dir/src/WGS84UTM.cc.s

CMakeFiles/my-module.dir/src/WGS84UTM.cc.o.requires:

.PHONY : CMakeFiles/my-module.dir/src/WGS84UTM.cc.o.requires

CMakeFiles/my-module.dir/src/WGS84UTM.cc.o.provides: CMakeFiles/my-module.dir/src/WGS84UTM.cc.o.requires
	$(MAKE) -f CMakeFiles/my-module.dir/build.make CMakeFiles/my-module.dir/src/WGS84UTM.cc.o.provides.build
.PHONY : CMakeFiles/my-module.dir/src/WGS84UTM.cc.o.provides

CMakeFiles/my-module.dir/src/WGS84UTM.cc.o.provides.build: CMakeFiles/my-module.dir/src/WGS84UTM.cc.o


CMakeFiles/my-module.dir/src/my_module.cc.o: CMakeFiles/my-module.dir/flags.make
CMakeFiles/my-module.dir/src/my_module.cc.o: ../src/my_module.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yxj/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/my-module.dir/src/my_module.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/my-module.dir/src/my_module.cc.o -c /home/yxj/Projects/txsim-zcm/src/my_module.cc

CMakeFiles/my-module.dir/src/my_module.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my-module.dir/src/my_module.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yxj/Projects/txsim-zcm/src/my_module.cc > CMakeFiles/my-module.dir/src/my_module.cc.i

CMakeFiles/my-module.dir/src/my_module.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my-module.dir/src/my_module.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yxj/Projects/txsim-zcm/src/my_module.cc -o CMakeFiles/my-module.dir/src/my_module.cc.s

CMakeFiles/my-module.dir/src/my_module.cc.o.requires:

.PHONY : CMakeFiles/my-module.dir/src/my_module.cc.o.requires

CMakeFiles/my-module.dir/src/my_module.cc.o.provides: CMakeFiles/my-module.dir/src/my_module.cc.o.requires
	$(MAKE) -f CMakeFiles/my-module.dir/build.make CMakeFiles/my-module.dir/src/my_module.cc.o.provides.build
.PHONY : CMakeFiles/my-module.dir/src/my_module.cc.o.provides

CMakeFiles/my-module.dir/src/my_module.cc.o.provides.build: CMakeFiles/my-module.dir/src/my_module.cc.o


CMakeFiles/my-module.dir/basic.pb.cc.o: CMakeFiles/my-module.dir/flags.make
CMakeFiles/my-module.dir/basic.pb.cc.o: basic.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yxj/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/my-module.dir/basic.pb.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/my-module.dir/basic.pb.cc.o -c /home/yxj/Projects/txsim-zcm/build/basic.pb.cc

CMakeFiles/my-module.dir/basic.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my-module.dir/basic.pb.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yxj/Projects/txsim-zcm/build/basic.pb.cc > CMakeFiles/my-module.dir/basic.pb.cc.i

CMakeFiles/my-module.dir/basic.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my-module.dir/basic.pb.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yxj/Projects/txsim-zcm/build/basic.pb.cc -o CMakeFiles/my-module.dir/basic.pb.cc.s

CMakeFiles/my-module.dir/basic.pb.cc.o.requires:

.PHONY : CMakeFiles/my-module.dir/basic.pb.cc.o.requires

CMakeFiles/my-module.dir/basic.pb.cc.o.provides: CMakeFiles/my-module.dir/basic.pb.cc.o.requires
	$(MAKE) -f CMakeFiles/my-module.dir/build.make CMakeFiles/my-module.dir/basic.pb.cc.o.provides.build
.PHONY : CMakeFiles/my-module.dir/basic.pb.cc.o.provides

CMakeFiles/my-module.dir/basic.pb.cc.o.provides.build: CMakeFiles/my-module.dir/basic.pb.cc.o


CMakeFiles/my-module.dir/header.pb.cc.o: CMakeFiles/my-module.dir/flags.make
CMakeFiles/my-module.dir/header.pb.cc.o: header.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yxj/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/my-module.dir/header.pb.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/my-module.dir/header.pb.cc.o -c /home/yxj/Projects/txsim-zcm/build/header.pb.cc

CMakeFiles/my-module.dir/header.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my-module.dir/header.pb.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yxj/Projects/txsim-zcm/build/header.pb.cc > CMakeFiles/my-module.dir/header.pb.cc.i

CMakeFiles/my-module.dir/header.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my-module.dir/header.pb.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yxj/Projects/txsim-zcm/build/header.pb.cc -o CMakeFiles/my-module.dir/header.pb.cc.s

CMakeFiles/my-module.dir/header.pb.cc.o.requires:

.PHONY : CMakeFiles/my-module.dir/header.pb.cc.o.requires

CMakeFiles/my-module.dir/header.pb.cc.o.provides: CMakeFiles/my-module.dir/header.pb.cc.o.requires
	$(MAKE) -f CMakeFiles/my-module.dir/build.make CMakeFiles/my-module.dir/header.pb.cc.o.provides.build
.PHONY : CMakeFiles/my-module.dir/header.pb.cc.o.provides

CMakeFiles/my-module.dir/header.pb.cc.o.provides.build: CMakeFiles/my-module.dir/header.pb.cc.o


CMakeFiles/my-module.dir/location.pb.cc.o: CMakeFiles/my-module.dir/flags.make
CMakeFiles/my-module.dir/location.pb.cc.o: location.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yxj/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object CMakeFiles/my-module.dir/location.pb.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/my-module.dir/location.pb.cc.o -c /home/yxj/Projects/txsim-zcm/build/location.pb.cc

CMakeFiles/my-module.dir/location.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my-module.dir/location.pb.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yxj/Projects/txsim-zcm/build/location.pb.cc > CMakeFiles/my-module.dir/location.pb.cc.i

CMakeFiles/my-module.dir/location.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my-module.dir/location.pb.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yxj/Projects/txsim-zcm/build/location.pb.cc -o CMakeFiles/my-module.dir/location.pb.cc.s

CMakeFiles/my-module.dir/location.pb.cc.o.requires:

.PHONY : CMakeFiles/my-module.dir/location.pb.cc.o.requires

CMakeFiles/my-module.dir/location.pb.cc.o.provides: CMakeFiles/my-module.dir/location.pb.cc.o.requires
	$(MAKE) -f CMakeFiles/my-module.dir/build.make CMakeFiles/my-module.dir/location.pb.cc.o.provides.build
.PHONY : CMakeFiles/my-module.dir/location.pb.cc.o.provides

CMakeFiles/my-module.dir/location.pb.cc.o.provides.build: CMakeFiles/my-module.dir/location.pb.cc.o


CMakeFiles/my-module.dir/trajectory.pb.cc.o: CMakeFiles/my-module.dir/flags.make
CMakeFiles/my-module.dir/trajectory.pb.cc.o: trajectory.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yxj/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Building CXX object CMakeFiles/my-module.dir/trajectory.pb.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/my-module.dir/trajectory.pb.cc.o -c /home/yxj/Projects/txsim-zcm/build/trajectory.pb.cc

CMakeFiles/my-module.dir/trajectory.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my-module.dir/trajectory.pb.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yxj/Projects/txsim-zcm/build/trajectory.pb.cc > CMakeFiles/my-module.dir/trajectory.pb.cc.i

CMakeFiles/my-module.dir/trajectory.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my-module.dir/trajectory.pb.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yxj/Projects/txsim-zcm/build/trajectory.pb.cc -o CMakeFiles/my-module.dir/trajectory.pb.cc.s

CMakeFiles/my-module.dir/trajectory.pb.cc.o.requires:

.PHONY : CMakeFiles/my-module.dir/trajectory.pb.cc.o.requires

CMakeFiles/my-module.dir/trajectory.pb.cc.o.provides: CMakeFiles/my-module.dir/trajectory.pb.cc.o.requires
	$(MAKE) -f CMakeFiles/my-module.dir/build.make CMakeFiles/my-module.dir/trajectory.pb.cc.o.provides.build
.PHONY : CMakeFiles/my-module.dir/trajectory.pb.cc.o.provides

CMakeFiles/my-module.dir/trajectory.pb.cc.o.provides.build: CMakeFiles/my-module.dir/trajectory.pb.cc.o


CMakeFiles/my-module.dir/planStatus.pb.cc.o: CMakeFiles/my-module.dir/flags.make
CMakeFiles/my-module.dir/planStatus.pb.cc.o: planStatus.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yxj/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Building CXX object CMakeFiles/my-module.dir/planStatus.pb.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/my-module.dir/planStatus.pb.cc.o -c /home/yxj/Projects/txsim-zcm/build/planStatus.pb.cc

CMakeFiles/my-module.dir/planStatus.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my-module.dir/planStatus.pb.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yxj/Projects/txsim-zcm/build/planStatus.pb.cc > CMakeFiles/my-module.dir/planStatus.pb.cc.i

CMakeFiles/my-module.dir/planStatus.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my-module.dir/planStatus.pb.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yxj/Projects/txsim-zcm/build/planStatus.pb.cc -o CMakeFiles/my-module.dir/planStatus.pb.cc.s

CMakeFiles/my-module.dir/planStatus.pb.cc.o.requires:

.PHONY : CMakeFiles/my-module.dir/planStatus.pb.cc.o.requires

CMakeFiles/my-module.dir/planStatus.pb.cc.o.provides: CMakeFiles/my-module.dir/planStatus.pb.cc.o.requires
	$(MAKE) -f CMakeFiles/my-module.dir/build.make CMakeFiles/my-module.dir/planStatus.pb.cc.o.provides.build
.PHONY : CMakeFiles/my-module.dir/planStatus.pb.cc.o.provides

CMakeFiles/my-module.dir/planStatus.pb.cc.o.provides.build: CMakeFiles/my-module.dir/planStatus.pb.cc.o


CMakeFiles/my-module.dir/laneMarks.pb.cc.o: CMakeFiles/my-module.dir/flags.make
CMakeFiles/my-module.dir/laneMarks.pb.cc.o: laneMarks.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yxj/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_18) "Building CXX object CMakeFiles/my-module.dir/laneMarks.pb.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/my-module.dir/laneMarks.pb.cc.o -c /home/yxj/Projects/txsim-zcm/build/laneMarks.pb.cc

CMakeFiles/my-module.dir/laneMarks.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my-module.dir/laneMarks.pb.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yxj/Projects/txsim-zcm/build/laneMarks.pb.cc > CMakeFiles/my-module.dir/laneMarks.pb.cc.i

CMakeFiles/my-module.dir/laneMarks.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my-module.dir/laneMarks.pb.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yxj/Projects/txsim-zcm/build/laneMarks.pb.cc -o CMakeFiles/my-module.dir/laneMarks.pb.cc.s

CMakeFiles/my-module.dir/laneMarks.pb.cc.o.requires:

.PHONY : CMakeFiles/my-module.dir/laneMarks.pb.cc.o.requires

CMakeFiles/my-module.dir/laneMarks.pb.cc.o.provides: CMakeFiles/my-module.dir/laneMarks.pb.cc.o.requires
	$(MAKE) -f CMakeFiles/my-module.dir/build.make CMakeFiles/my-module.dir/laneMarks.pb.cc.o.provides.build
.PHONY : CMakeFiles/my-module.dir/laneMarks.pb.cc.o.provides

CMakeFiles/my-module.dir/laneMarks.pb.cc.o.provides.build: CMakeFiles/my-module.dir/laneMarks.pb.cc.o


CMakeFiles/my-module.dir/traffic.pb.cc.o: CMakeFiles/my-module.dir/flags.make
CMakeFiles/my-module.dir/traffic.pb.cc.o: traffic.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yxj/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_19) "Building CXX object CMakeFiles/my-module.dir/traffic.pb.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/my-module.dir/traffic.pb.cc.o -c /home/yxj/Projects/txsim-zcm/build/traffic.pb.cc

CMakeFiles/my-module.dir/traffic.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my-module.dir/traffic.pb.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yxj/Projects/txsim-zcm/build/traffic.pb.cc > CMakeFiles/my-module.dir/traffic.pb.cc.i

CMakeFiles/my-module.dir/traffic.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my-module.dir/traffic.pb.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yxj/Projects/txsim-zcm/build/traffic.pb.cc -o CMakeFiles/my-module.dir/traffic.pb.cc.s

CMakeFiles/my-module.dir/traffic.pb.cc.o.requires:

.PHONY : CMakeFiles/my-module.dir/traffic.pb.cc.o.requires

CMakeFiles/my-module.dir/traffic.pb.cc.o.provides: CMakeFiles/my-module.dir/traffic.pb.cc.o.requires
	$(MAKE) -f CMakeFiles/my-module.dir/build.make CMakeFiles/my-module.dir/traffic.pb.cc.o.provides.build
.PHONY : CMakeFiles/my-module.dir/traffic.pb.cc.o.provides

CMakeFiles/my-module.dir/traffic.pb.cc.o.provides.build: CMakeFiles/my-module.dir/traffic.pb.cc.o


CMakeFiles/my-module.dir/control.pb.cc.o: CMakeFiles/my-module.dir/flags.make
CMakeFiles/my-module.dir/control.pb.cc.o: control.pb.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yxj/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_20) "Building CXX object CMakeFiles/my-module.dir/control.pb.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/my-module.dir/control.pb.cc.o -c /home/yxj/Projects/txsim-zcm/build/control.pb.cc

CMakeFiles/my-module.dir/control.pb.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my-module.dir/control.pb.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yxj/Projects/txsim-zcm/build/control.pb.cc > CMakeFiles/my-module.dir/control.pb.cc.i

CMakeFiles/my-module.dir/control.pb.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my-module.dir/control.pb.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yxj/Projects/txsim-zcm/build/control.pb.cc -o CMakeFiles/my-module.dir/control.pb.cc.s

CMakeFiles/my-module.dir/control.pb.cc.o.requires:

.PHONY : CMakeFiles/my-module.dir/control.pb.cc.o.requires

CMakeFiles/my-module.dir/control.pb.cc.o.provides: CMakeFiles/my-module.dir/control.pb.cc.o.requires
	$(MAKE) -f CMakeFiles/my-module.dir/build.make CMakeFiles/my-module.dir/control.pb.cc.o.provides.build
.PHONY : CMakeFiles/my-module.dir/control.pb.cc.o.provides

CMakeFiles/my-module.dir/control.pb.cc.o.provides.build: CMakeFiles/my-module.dir/control.pb.cc.o


# Object files for target my-module
my__module_OBJECTS = \
"CMakeFiles/my-module.dir/src/MessageManager.cc.o" \
"CMakeFiles/my-module.dir/src/MessageManagerBase.cc.o" \
"CMakeFiles/my-module.dir/src/WGS84UTM.cc.o" \
"CMakeFiles/my-module.dir/src/my_module.cc.o" \
"CMakeFiles/my-module.dir/basic.pb.cc.o" \
"CMakeFiles/my-module.dir/header.pb.cc.o" \
"CMakeFiles/my-module.dir/location.pb.cc.o" \
"CMakeFiles/my-module.dir/trajectory.pb.cc.o" \
"CMakeFiles/my-module.dir/planStatus.pb.cc.o" \
"CMakeFiles/my-module.dir/laneMarks.pb.cc.o" \
"CMakeFiles/my-module.dir/traffic.pb.cc.o" \
"CMakeFiles/my-module.dir/control.pb.cc.o"

# External object files for target my-module
my__module_EXTERNAL_OBJECTS =

lib/libmy-module.so: CMakeFiles/my-module.dir/src/MessageManager.cc.o
lib/libmy-module.so: CMakeFiles/my-module.dir/src/MessageManagerBase.cc.o
lib/libmy-module.so: CMakeFiles/my-module.dir/src/WGS84UTM.cc.o
lib/libmy-module.so: CMakeFiles/my-module.dir/src/my_module.cc.o
lib/libmy-module.so: CMakeFiles/my-module.dir/basic.pb.cc.o
lib/libmy-module.so: CMakeFiles/my-module.dir/header.pb.cc.o
lib/libmy-module.so: CMakeFiles/my-module.dir/location.pb.cc.o
lib/libmy-module.so: CMakeFiles/my-module.dir/trajectory.pb.cc.o
lib/libmy-module.so: CMakeFiles/my-module.dir/planStatus.pb.cc.o
lib/libmy-module.so: CMakeFiles/my-module.dir/laneMarks.pb.cc.o
lib/libmy-module.so: CMakeFiles/my-module.dir/traffic.pb.cc.o
lib/libmy-module.so: CMakeFiles/my-module.dir/control.pb.cc.o
lib/libmy-module.so: CMakeFiles/my-module.dir/build.make
lib/libmy-module.so: CMakeFiles/my-module.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yxj/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_21) "Linking CXX shared library lib/libmy-module.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/my-module.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/my-module.dir/build: lib/libmy-module.so

.PHONY : CMakeFiles/my-module.dir/build

CMakeFiles/my-module.dir/requires: CMakeFiles/my-module.dir/src/MessageManager.cc.o.requires
CMakeFiles/my-module.dir/requires: CMakeFiles/my-module.dir/src/MessageManagerBase.cc.o.requires
CMakeFiles/my-module.dir/requires: CMakeFiles/my-module.dir/src/WGS84UTM.cc.o.requires
CMakeFiles/my-module.dir/requires: CMakeFiles/my-module.dir/src/my_module.cc.o.requires
CMakeFiles/my-module.dir/requires: CMakeFiles/my-module.dir/basic.pb.cc.o.requires
CMakeFiles/my-module.dir/requires: CMakeFiles/my-module.dir/header.pb.cc.o.requires
CMakeFiles/my-module.dir/requires: CMakeFiles/my-module.dir/location.pb.cc.o.requires
CMakeFiles/my-module.dir/requires: CMakeFiles/my-module.dir/trajectory.pb.cc.o.requires
CMakeFiles/my-module.dir/requires: CMakeFiles/my-module.dir/planStatus.pb.cc.o.requires
CMakeFiles/my-module.dir/requires: CMakeFiles/my-module.dir/laneMarks.pb.cc.o.requires
CMakeFiles/my-module.dir/requires: CMakeFiles/my-module.dir/traffic.pb.cc.o.requires
CMakeFiles/my-module.dir/requires: CMakeFiles/my-module.dir/control.pb.cc.o.requires

.PHONY : CMakeFiles/my-module.dir/requires

CMakeFiles/my-module.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/my-module.dir/cmake_clean.cmake
.PHONY : CMakeFiles/my-module.dir/clean

CMakeFiles/my-module.dir/depend: basic.pb.cc
CMakeFiles/my-module.dir/depend: basic.pb.h
CMakeFiles/my-module.dir/depend: header.pb.cc
CMakeFiles/my-module.dir/depend: header.pb.h
CMakeFiles/my-module.dir/depend: location.pb.cc
CMakeFiles/my-module.dir/depend: location.pb.h
CMakeFiles/my-module.dir/depend: trajectory.pb.cc
CMakeFiles/my-module.dir/depend: trajectory.pb.h
CMakeFiles/my-module.dir/depend: planStatus.pb.cc
CMakeFiles/my-module.dir/depend: planStatus.pb.h
CMakeFiles/my-module.dir/depend: laneMarks.pb.cc
CMakeFiles/my-module.dir/depend: laneMarks.pb.h
CMakeFiles/my-module.dir/depend: traffic.pb.cc
CMakeFiles/my-module.dir/depend: traffic.pb.h
CMakeFiles/my-module.dir/depend: control.pb.cc
CMakeFiles/my-module.dir/depend: control.pb.h
	cd /home/yxj/Projects/txsim-zcm/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yxj/Projects/txsim-zcm /home/yxj/Projects/txsim-zcm /home/yxj/Projects/txsim-zcm/build /home/yxj/Projects/txsim-zcm/build /home/yxj/Projects/txsim-zcm/build/CMakeFiles/my-module.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/my-module.dir/depend

