# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_COMMAND = /home/steve/Tools/clion-2017.1/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/steve/Tools/clion-2017.1/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/steve/Code/MagnetometerFeatureAugumentationLocation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/steve/Code/MagnetometerFeatureAugumentationLocation/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/graph_build.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/graph_build.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/graph_build.dir/flags.make

CMakeFiles/graph_build.dir/src/graph_build_test.cpp.o: CMakeFiles/graph_build.dir/flags.make
CMakeFiles/graph_build.dir/src/graph_build_test.cpp.o: ../src/graph_build_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/steve/Code/MagnetometerFeatureAugumentationLocation/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/graph_build.dir/src/graph_build_test.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/graph_build.dir/src/graph_build_test.cpp.o -c /home/steve/Code/MagnetometerFeatureAugumentationLocation/src/graph_build_test.cpp

CMakeFiles/graph_build.dir/src/graph_build_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/graph_build.dir/src/graph_build_test.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/steve/Code/MagnetometerFeatureAugumentationLocation/src/graph_build_test.cpp > CMakeFiles/graph_build.dir/src/graph_build_test.cpp.i

CMakeFiles/graph_build.dir/src/graph_build_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/graph_build.dir/src/graph_build_test.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/steve/Code/MagnetometerFeatureAugumentationLocation/src/graph_build_test.cpp -o CMakeFiles/graph_build.dir/src/graph_build_test.cpp.s

CMakeFiles/graph_build.dir/src/graph_build_test.cpp.o.requires:

.PHONY : CMakeFiles/graph_build.dir/src/graph_build_test.cpp.o.requires

CMakeFiles/graph_build.dir/src/graph_build_test.cpp.o.provides: CMakeFiles/graph_build.dir/src/graph_build_test.cpp.o.requires
	$(MAKE) -f CMakeFiles/graph_build.dir/build.make CMakeFiles/graph_build.dir/src/graph_build_test.cpp.o.provides.build
.PHONY : CMakeFiles/graph_build.dir/src/graph_build_test.cpp.o.provides

CMakeFiles/graph_build.dir/src/graph_build_test.cpp.o.provides.build: CMakeFiles/graph_build.dir/src/graph_build_test.cpp.o


# Object files for target graph_build
graph_build_OBJECTS = \
"CMakeFiles/graph_build.dir/src/graph_build_test.cpp.o"

# External object files for target graph_build
graph_build_EXTERNAL_OBJECTS =

graph_build: CMakeFiles/graph_build.dir/src/graph_build_test.cpp.o
graph_build: CMakeFiles/graph_build.dir/build.make
graph_build: /usr/lib/x86_64-linux-gnu/libcxsparse.so
graph_build: /usr/lib/x86_64-linux-gnu/libpython2.7.so
graph_build: CMakeFiles/graph_build.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/steve/Code/MagnetometerFeatureAugumentationLocation/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable graph_build"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/graph_build.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/graph_build.dir/build: graph_build

.PHONY : CMakeFiles/graph_build.dir/build

CMakeFiles/graph_build.dir/requires: CMakeFiles/graph_build.dir/src/graph_build_test.cpp.o.requires

.PHONY : CMakeFiles/graph_build.dir/requires

CMakeFiles/graph_build.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/graph_build.dir/cmake_clean.cmake
.PHONY : CMakeFiles/graph_build.dir/clean

CMakeFiles/graph_build.dir/depend:
	cd /home/steve/Code/MagnetometerFeatureAugumentationLocation/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/steve/Code/MagnetometerFeatureAugumentationLocation /home/steve/Code/MagnetometerFeatureAugumentationLocation /home/steve/Code/MagnetometerFeatureAugumentationLocation/cmake-build-debug /home/steve/Code/MagnetometerFeatureAugumentationLocation/cmake-build-debug /home/steve/Code/MagnetometerFeatureAugumentationLocation/cmake-build-debug/CMakeFiles/graph_build.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/graph_build.dir/depend

