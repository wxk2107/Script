# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.12

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = "/home/damao/Documents/Script/collision_checking "

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/damao/Documents/Script/collision_checking "

# Include any dependencies generated for this target.
include CMakeFiles/collision_checker.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/collision_checker.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/collision_checker.dir/flags.make

CMakeFiles/collision_checker.dir/src/test.cpp.o: CMakeFiles/collision_checker.dir/flags.make
CMakeFiles/collision_checker.dir/src/test.cpp.o: src/test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/damao/Documents/Script/collision_checking /CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/collision_checker.dir/src/test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/collision_checker.dir/src/test.cpp.o -c "/home/damao/Documents/Script/collision_checking /src/test.cpp"

CMakeFiles/collision_checker.dir/src/test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/collision_checker.dir/src/test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/damao/Documents/Script/collision_checking /src/test.cpp" > CMakeFiles/collision_checker.dir/src/test.cpp.i

CMakeFiles/collision_checker.dir/src/test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/collision_checker.dir/src/test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/damao/Documents/Script/collision_checking /src/test.cpp" -o CMakeFiles/collision_checker.dir/src/test.cpp.s

CMakeFiles/collision_checker.dir/src/collision_checking.cpp.o: CMakeFiles/collision_checker.dir/flags.make
CMakeFiles/collision_checker.dir/src/collision_checking.cpp.o: src/collision_checking.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/damao/Documents/Script/collision_checking /CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/collision_checker.dir/src/collision_checking.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/collision_checker.dir/src/collision_checking.cpp.o -c "/home/damao/Documents/Script/collision_checking /src/collision_checking.cpp"

CMakeFiles/collision_checker.dir/src/collision_checking.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/collision_checker.dir/src/collision_checking.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/damao/Documents/Script/collision_checking /src/collision_checking.cpp" > CMakeFiles/collision_checker.dir/src/collision_checking.cpp.i

CMakeFiles/collision_checker.dir/src/collision_checking.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/collision_checker.dir/src/collision_checking.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/damao/Documents/Script/collision_checking /src/collision_checking.cpp" -o CMakeFiles/collision_checker.dir/src/collision_checking.cpp.s

# Object files for target collision_checker
collision_checker_OBJECTS = \
"CMakeFiles/collision_checker.dir/src/test.cpp.o" \
"CMakeFiles/collision_checker.dir/src/collision_checking.cpp.o"

# External object files for target collision_checker
collision_checker_EXTERNAL_OBJECTS =

collision_checker: CMakeFiles/collision_checker.dir/src/test.cpp.o
collision_checker: CMakeFiles/collision_checker.dir/src/collision_checking.cpp.o
collision_checker: CMakeFiles/collision_checker.dir/build.make
collision_checker: /usr/lib/x86_64-linux-gnu/libboost_system.so
collision_checker: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
collision_checker: /usr/lib/x86_64-linux-gnu/libboost_thread.so
collision_checker: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
collision_checker: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
collision_checker: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
collision_checker: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
collision_checker: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
collision_checker: /usr/lib/x86_64-linux-gnu/libboost_regex.so
collision_checker: /usr/local/lib/libpcl_common.so
collision_checker: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
collision_checker: /usr/local/lib/libpcl_kdtree.so
collision_checker: /usr/local/lib/libpcl_octree.so
collision_checker: /usr/local/lib/libpcl_search.so
collision_checker: /usr/local/lib/libpcl_sample_consensus.so
collision_checker: /usr/local/lib/libpcl_filters.so
collision_checker: /usr/local/lib/libpcl_features.so
collision_checker: /usr/local/lib/libpcl_ml.so
collision_checker: /usr/local/lib/libpcl_segmentation.so
collision_checker: /usr/lib/libOpenNI.so
collision_checker: /usr/local/lib/libpcl_io.so
collision_checker: /usr/lib/x86_64-linux-gnu/libqhull.so
collision_checker: /usr/local/lib/libpcl_surface.so
collision_checker: /usr/local/lib/libpcl_visualization.so
collision_checker: /usr/local/lib/libpcl_keypoints.so
collision_checker: /usr/local/lib/libpcl_outofcore.so
collision_checker: /usr/local/lib/libpcl_stereo.so
collision_checker: /usr/local/lib/libpcl_registration.so
collision_checker: /usr/local/lib/libpcl_recognition.so
collision_checker: /usr/local/lib/libpcl_people.so
collision_checker: /usr/local/lib/libpcl_tracking.so
collision_checker: /usr/lib/x86_64-linux-gnu/libboost_system.so
collision_checker: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
collision_checker: /usr/lib/x86_64-linux-gnu/libboost_thread.so
collision_checker: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
collision_checker: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
collision_checker: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
collision_checker: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
collision_checker: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
collision_checker: /usr/lib/x86_64-linux-gnu/libboost_regex.so
collision_checker: /usr/lib/x86_64-linux-gnu/libqhull.so
collision_checker: /usr/lib/libOpenNI.so
collision_checker: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
collision_checker: /usr/lib/libvtkGenericFiltering.so.5.8.0
collision_checker: /usr/lib/libvtkGeovis.so.5.8.0
collision_checker: /usr/lib/libvtkCharts.so.5.8.0
collision_checker: /usr/local/lib/libpcl_common.so
collision_checker: /usr/local/lib/libpcl_kdtree.so
collision_checker: /usr/local/lib/libpcl_octree.so
collision_checker: /usr/local/lib/libpcl_search.so
collision_checker: /usr/local/lib/libpcl_sample_consensus.so
collision_checker: /usr/local/lib/libpcl_filters.so
collision_checker: /usr/local/lib/libpcl_features.so
collision_checker: /usr/local/lib/libpcl_ml.so
collision_checker: /usr/local/lib/libpcl_segmentation.so
collision_checker: /usr/local/lib/libpcl_io.so
collision_checker: /usr/local/lib/libpcl_surface.so
collision_checker: /usr/local/lib/libpcl_visualization.so
collision_checker: /usr/local/lib/libpcl_keypoints.so
collision_checker: /usr/local/lib/libpcl_outofcore.so
collision_checker: /usr/local/lib/libpcl_stereo.so
collision_checker: /usr/local/lib/libpcl_registration.so
collision_checker: /usr/local/lib/libpcl_recognition.so
collision_checker: /usr/local/lib/libpcl_people.so
collision_checker: /usr/local/lib/libpcl_tracking.so
collision_checker: /usr/lib/libvtkViews.so.5.8.0
collision_checker: /usr/lib/libvtkInfovis.so.5.8.0
collision_checker: /usr/lib/libvtkWidgets.so.5.8.0
collision_checker: /usr/lib/libvtkVolumeRendering.so.5.8.0
collision_checker: /usr/lib/libvtkHybrid.so.5.8.0
collision_checker: /usr/lib/libvtkParallel.so.5.8.0
collision_checker: /usr/lib/libvtkRendering.so.5.8.0
collision_checker: /usr/lib/libvtkImaging.so.5.8.0
collision_checker: /usr/lib/libvtkGraphics.so.5.8.0
collision_checker: /usr/lib/libvtkIO.so.5.8.0
collision_checker: /usr/lib/libvtkFiltering.so.5.8.0
collision_checker: /usr/lib/libvtkCommon.so.5.8.0
collision_checker: /usr/lib/libvtksys.so.5.8.0
collision_checker: CMakeFiles/collision_checker.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/damao/Documents/Script/collision_checking /CMakeFiles" --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable collision_checker"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/collision_checker.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/collision_checker.dir/build: collision_checker

.PHONY : CMakeFiles/collision_checker.dir/build

CMakeFiles/collision_checker.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/collision_checker.dir/cmake_clean.cmake
.PHONY : CMakeFiles/collision_checker.dir/clean

CMakeFiles/collision_checker.dir/depend:
	cd "/home/damao/Documents/Script/collision_checking " && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/damao/Documents/Script/collision_checking " "/home/damao/Documents/Script/collision_checking " "/home/damao/Documents/Script/collision_checking " "/home/damao/Documents/Script/collision_checking " "/home/damao/Documents/Script/collision_checking /CMakeFiles/collision_checker.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/collision_checker.dir/depend

