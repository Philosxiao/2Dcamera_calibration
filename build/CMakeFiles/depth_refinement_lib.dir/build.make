# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/philos/Desktop/cam_catch

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/philos/Desktop/cam_catch/build

# Include any dependencies generated for this target.
include CMakeFiles/depth_refinement_lib.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/depth_refinement_lib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/depth_refinement_lib.dir/flags.make

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/bilateralFilter.cpp.o: CMakeFiles/depth_refinement_lib.dir/flags.make
CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/bilateralFilter.cpp.o: ../depth_map_refinement/bilateralFilter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/philos/Desktop/cam_catch/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/bilateralFilter.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/bilateralFilter.cpp.o -c /home/philos/Desktop/cam_catch/depth_map_refinement/bilateralFilter.cpp

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/bilateralFilter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/bilateralFilter.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/philos/Desktop/cam_catch/depth_map_refinement/bilateralFilter.cpp > CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/bilateralFilter.cpp.i

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/bilateralFilter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/bilateralFilter.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/philos/Desktop/cam_catch/depth_map_refinement/bilateralFilter.cpp -o CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/bilateralFilter.cpp.s

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/bilateralFilter.cpp.o.requires:

.PHONY : CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/bilateralFilter.cpp.o.requires

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/bilateralFilter.cpp.o.provides: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/bilateralFilter.cpp.o.requires
	$(MAKE) -f CMakeFiles/depth_refinement_lib.dir/build.make CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/bilateralFilter.cpp.o.provides.build
.PHONY : CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/bilateralFilter.cpp.o.provides

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/bilateralFilter.cpp.o.provides.build: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/bilateralFilter.cpp.o


CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/depthFilter.cpp.o: CMakeFiles/depth_refinement_lib.dir/flags.make
CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/depthFilter.cpp.o: ../depth_map_refinement/depthFilter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/philos/Desktop/cam_catch/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/depthFilter.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/depthFilter.cpp.o -c /home/philos/Desktop/cam_catch/depth_map_refinement/depthFilter.cpp

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/depthFilter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/depthFilter.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/philos/Desktop/cam_catch/depth_map_refinement/depthFilter.cpp > CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/depthFilter.cpp.i

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/depthFilter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/depthFilter.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/philos/Desktop/cam_catch/depth_map_refinement/depthFilter.cpp -o CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/depthFilter.cpp.s

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/depthFilter.cpp.o.requires:

.PHONY : CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/depthFilter.cpp.o.requires

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/depthFilter.cpp.o.provides: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/depthFilter.cpp.o.requires
	$(MAKE) -f CMakeFiles/depth_refinement_lib.dir/build.make CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/depthFilter.cpp.o.provides.build
.PHONY : CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/depthFilter.cpp.o.provides

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/depthFilter.cpp.o.provides.build: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/depthFilter.cpp.o


CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointBilateralFilter.cpp.o: CMakeFiles/depth_refinement_lib.dir/flags.make
CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointBilateralFilter.cpp.o: ../depth_map_refinement/jointBilateralFilter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/philos/Desktop/cam_catch/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointBilateralFilter.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointBilateralFilter.cpp.o -c /home/philos/Desktop/cam_catch/depth_map_refinement/jointBilateralFilter.cpp

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointBilateralFilter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointBilateralFilter.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/philos/Desktop/cam_catch/depth_map_refinement/jointBilateralFilter.cpp > CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointBilateralFilter.cpp.i

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointBilateralFilter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointBilateralFilter.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/philos/Desktop/cam_catch/depth_map_refinement/jointBilateralFilter.cpp -o CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointBilateralFilter.cpp.s

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointBilateralFilter.cpp.o.requires:

.PHONY : CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointBilateralFilter.cpp.o.requires

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointBilateralFilter.cpp.o.provides: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointBilateralFilter.cpp.o.requires
	$(MAKE) -f CMakeFiles/depth_refinement_lib.dir/build.make CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointBilateralFilter.cpp.o.provides.build
.PHONY : CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointBilateralFilter.cpp.o.provides

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointBilateralFilter.cpp.o.provides.build: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointBilateralFilter.cpp.o


CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointNearest.cpp.o: CMakeFiles/depth_refinement_lib.dir/flags.make
CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointNearest.cpp.o: ../depth_map_refinement/jointNearest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/philos/Desktop/cam_catch/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointNearest.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointNearest.cpp.o -c /home/philos/Desktop/cam_catch/depth_map_refinement/jointNearest.cpp

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointNearest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointNearest.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/philos/Desktop/cam_catch/depth_map_refinement/jointNearest.cpp > CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointNearest.cpp.i

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointNearest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointNearest.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/philos/Desktop/cam_catch/depth_map_refinement/jointNearest.cpp -o CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointNearest.cpp.s

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointNearest.cpp.o.requires:

.PHONY : CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointNearest.cpp.o.requires

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointNearest.cpp.o.provides: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointNearest.cpp.o.requires
	$(MAKE) -f CMakeFiles/depth_refinement_lib.dir/build.make CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointNearest.cpp.o.provides.build
.PHONY : CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointNearest.cpp.o.provides

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointNearest.cpp.o.provides.build: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointNearest.cpp.o


CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/trilateralFilter.cpp.o: CMakeFiles/depth_refinement_lib.dir/flags.make
CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/trilateralFilter.cpp.o: ../depth_map_refinement/trilateralFilter.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/philos/Desktop/cam_catch/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/trilateralFilter.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/trilateralFilter.cpp.o -c /home/philos/Desktop/cam_catch/depth_map_refinement/trilateralFilter.cpp

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/trilateralFilter.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/trilateralFilter.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/philos/Desktop/cam_catch/depth_map_refinement/trilateralFilter.cpp > CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/trilateralFilter.cpp.i

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/trilateralFilter.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/trilateralFilter.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/philos/Desktop/cam_catch/depth_map_refinement/trilateralFilter.cpp -o CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/trilateralFilter.cpp.s

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/trilateralFilter.cpp.o.requires:

.PHONY : CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/trilateralFilter.cpp.o.requires

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/trilateralFilter.cpp.o.provides: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/trilateralFilter.cpp.o.requires
	$(MAKE) -f CMakeFiles/depth_refinement_lib.dir/build.make CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/trilateralFilter.cpp.o.provides.build
.PHONY : CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/trilateralFilter.cpp.o.provides

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/trilateralFilter.cpp.o.provides.build: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/trilateralFilter.cpp.o


CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/split.cpp.o: CMakeFiles/depth_refinement_lib.dir/flags.make
CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/split.cpp.o: ../depth_map_refinement/split.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/philos/Desktop/cam_catch/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/split.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/split.cpp.o -c /home/philos/Desktop/cam_catch/depth_map_refinement/split.cpp

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/split.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/split.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/philos/Desktop/cam_catch/depth_map_refinement/split.cpp > CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/split.cpp.i

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/split.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/split.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/philos/Desktop/cam_catch/depth_map_refinement/split.cpp -o CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/split.cpp.s

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/split.cpp.o.requires:

.PHONY : CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/split.cpp.o.requires

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/split.cpp.o.provides: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/split.cpp.o.requires
	$(MAKE) -f CMakeFiles/depth_refinement_lib.dir/build.make CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/split.cpp.o.provides.build
.PHONY : CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/split.cpp.o.provides

CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/split.cpp.o.provides.build: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/split.cpp.o


# Object files for target depth_refinement_lib
depth_refinement_lib_OBJECTS = \
"CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/bilateralFilter.cpp.o" \
"CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/depthFilter.cpp.o" \
"CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointBilateralFilter.cpp.o" \
"CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointNearest.cpp.o" \
"CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/trilateralFilter.cpp.o" \
"CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/split.cpp.o"

# External object files for target depth_refinement_lib
depth_refinement_lib_EXTERNAL_OBJECTS =

libdepth_refinement_lib.a: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/bilateralFilter.cpp.o
libdepth_refinement_lib.a: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/depthFilter.cpp.o
libdepth_refinement_lib.a: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointBilateralFilter.cpp.o
libdepth_refinement_lib.a: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointNearest.cpp.o
libdepth_refinement_lib.a: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/trilateralFilter.cpp.o
libdepth_refinement_lib.a: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/split.cpp.o
libdepth_refinement_lib.a: CMakeFiles/depth_refinement_lib.dir/build.make
libdepth_refinement_lib.a: CMakeFiles/depth_refinement_lib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/philos/Desktop/cam_catch/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX static library libdepth_refinement_lib.a"
	$(CMAKE_COMMAND) -P CMakeFiles/depth_refinement_lib.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/depth_refinement_lib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/depth_refinement_lib.dir/build: libdepth_refinement_lib.a

.PHONY : CMakeFiles/depth_refinement_lib.dir/build

CMakeFiles/depth_refinement_lib.dir/requires: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/bilateralFilter.cpp.o.requires
CMakeFiles/depth_refinement_lib.dir/requires: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/depthFilter.cpp.o.requires
CMakeFiles/depth_refinement_lib.dir/requires: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointBilateralFilter.cpp.o.requires
CMakeFiles/depth_refinement_lib.dir/requires: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/jointNearest.cpp.o.requires
CMakeFiles/depth_refinement_lib.dir/requires: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/trilateralFilter.cpp.o.requires
CMakeFiles/depth_refinement_lib.dir/requires: CMakeFiles/depth_refinement_lib.dir/depth_map_refinement/split.cpp.o.requires

.PHONY : CMakeFiles/depth_refinement_lib.dir/requires

CMakeFiles/depth_refinement_lib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/depth_refinement_lib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/depth_refinement_lib.dir/clean

CMakeFiles/depth_refinement_lib.dir/depend:
	cd /home/philos/Desktop/cam_catch/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/philos/Desktop/cam_catch /home/philos/Desktop/cam_catch /home/philos/Desktop/cam_catch/build /home/philos/Desktop/cam_catch/build /home/philos/Desktop/cam_catch/build/CMakeFiles/depth_refinement_lib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/depth_refinement_lib.dir/depend

