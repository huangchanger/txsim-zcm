# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/chang/Projects/txsim-zcm

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/chang/Projects/txsim-zcm/build

# Include any dependencies generated for this target.
include CMakeFiles/my-module.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/my-module.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/my-module.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/my-module.dir/flags.make

CMakeFiles/my-module.dir/src/my_module.cpp.o: CMakeFiles/my-module.dir/flags.make
CMakeFiles/my-module.dir/src/my_module.cpp.o: ../src/my_module.cpp
CMakeFiles/my-module.dir/src/my_module.cpp.o: CMakeFiles/my-module.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/chang/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/my-module.dir/src/my_module.cpp.o"
	/usr/bin/g++-5 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/my-module.dir/src/my_module.cpp.o -MF CMakeFiles/my-module.dir/src/my_module.cpp.o.d -o CMakeFiles/my-module.dir/src/my_module.cpp.o -c /home/chang/Projects/txsim-zcm/src/my_module.cpp

CMakeFiles/my-module.dir/src/my_module.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/my-module.dir/src/my_module.cpp.i"
	/usr/bin/g++-5 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/chang/Projects/txsim-zcm/src/my_module.cpp > CMakeFiles/my-module.dir/src/my_module.cpp.i

CMakeFiles/my-module.dir/src/my_module.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/my-module.dir/src/my_module.cpp.s"
	/usr/bin/g++-5 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/chang/Projects/txsim-zcm/src/my_module.cpp -o CMakeFiles/my-module.dir/src/my_module.cpp.s

# Object files for target my-module
my__module_OBJECTS = \
"CMakeFiles/my-module.dir/src/my_module.cpp.o"

# External object files for target my-module
my__module_EXTERNAL_OBJECTS =

lib/libmy-module.so: CMakeFiles/my-module.dir/src/my_module.cpp.o
lib/libmy-module.so: CMakeFiles/my-module.dir/build.make
lib/libmy-module.so: CMakeFiles/my-module.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/chang/Projects/txsim-zcm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library lib/libmy-module.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/my-module.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/my-module.dir/build: lib/libmy-module.so
.PHONY : CMakeFiles/my-module.dir/build

CMakeFiles/my-module.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/my-module.dir/cmake_clean.cmake
.PHONY : CMakeFiles/my-module.dir/clean

CMakeFiles/my-module.dir/depend:
	cd /home/chang/Projects/txsim-zcm/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chang/Projects/txsim-zcm /home/chang/Projects/txsim-zcm /home/chang/Projects/txsim-zcm/build /home/chang/Projects/txsim-zcm/build /home/chang/Projects/txsim-zcm/build/CMakeFiles/my-module.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/my-module.dir/depend

