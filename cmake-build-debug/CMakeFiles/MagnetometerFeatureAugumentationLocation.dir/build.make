# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.8

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
CMAKE_COMMAND = /home/steve/Tools/clion-2017.2.2/bin/cmake/bin/cmake

# The command to remove a file.
RM = /home/steve/Tools/clion-2017.2.2/bin/cmake/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/steve/Code/MagnetometerFeatureAugumentationLocation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/steve/Code/MagnetometerFeatureAugumentationLocation/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/flags.make

CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/main.cpp.o: CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/flags.make
CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/steve/Code/MagnetometerFeatureAugumentationLocation/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/main.cpp.o -c /home/steve/Code/MagnetometerFeatureAugumentationLocation/main.cpp

CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/steve/Code/MagnetometerFeatureAugumentationLocation/main.cpp > CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/main.cpp.i

CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/steve/Code/MagnetometerFeatureAugumentationLocation/main.cpp -o CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/main.cpp.s

CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/main.cpp.o.requires

CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/main.cpp.o.provides: CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/build.make CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/main.cpp.o.provides

CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/main.cpp.o.provides.build: CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/main.cpp.o


# Object files for target MagnetometerFeatureAugumentationLocation
MagnetometerFeatureAugumentationLocation_OBJECTS = \
"CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/main.cpp.o"

# External object files for target MagnetometerFeatureAugumentationLocation
MagnetometerFeatureAugumentationLocation_EXTERNAL_OBJECTS =

MagnetometerFeatureAugumentationLocation: CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/main.cpp.o
MagnetometerFeatureAugumentationLocation: CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/build.make
MagnetometerFeatureAugumentationLocation: CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/steve/Code/MagnetometerFeatureAugumentationLocation/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable MagnetometerFeatureAugumentationLocation"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/build: MagnetometerFeatureAugumentationLocation

.PHONY : CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/build

CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/requires: CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/main.cpp.o.requires

.PHONY : CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/requires

CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/clean

CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/depend:
	cd /home/steve/Code/MagnetometerFeatureAugumentationLocation/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/steve/Code/MagnetometerFeatureAugumentationLocation /home/steve/Code/MagnetometerFeatureAugumentationLocation /home/steve/Code/MagnetometerFeatureAugumentationLocation/cmake-build-debug /home/steve/Code/MagnetometerFeatureAugumentationLocation/cmake-build-debug /home/steve/Code/MagnetometerFeatureAugumentationLocation/cmake-build-debug/CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/MagnetometerFeatureAugumentationLocation.dir/depend

