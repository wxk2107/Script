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
CMAKE_SOURCE_DIR = /home/damao/Documents/Script/pcl_test/pose_estimation

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/damao/Documents/Script/pcl_test/pose_estimation

# Include any dependencies generated for this target.
include CMakeFiles/alignment_prerejective.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/alignment_prerejective.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/alignment_prerejective.dir/flags.make

CMakeFiles/alignment_prerejective.dir/alignment_prerejective.cpp.o: CMakeFiles/alignment_prerejective.dir/flags.make
CMakeFiles/alignment_prerejective.dir/alignment_prerejective.cpp.o: alignment_prerejective.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/damao/Documents/Script/pcl_test/pose_estimation/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/alignment_prerejective.dir/alignment_prerejective.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/alignment_prerejective.dir/alignment_prerejective.cpp.o -c /home/damao/Documents/Script/pcl_test/pose_estimation/alignment_prerejective.cpp

CMakeFiles/alignment_prerejective.dir/alignment_prerejective.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/alignment_prerejective.dir/alignment_prerejective.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/damao/Documents/Script/pcl_test/pose_estimation/alignment_prerejective.cpp > CMakeFiles/alignment_prerejective.dir/alignment_prerejective.cpp.i

CMakeFiles/alignment_prerejective.dir/alignment_prerejective.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/alignment_prerejective.dir/alignment_prerejective.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/damao/Documents/Script/pcl_test/pose_estimation/alignment_prerejective.cpp -o CMakeFiles/alignment_prerejective.dir/alignment_prerejective.cpp.s

# Object files for target alignment_prerejective
alignment_prerejective_OBJECTS = \
"CMakeFiles/alignment_prerejective.dir/alignment_prerejective.cpp.o"

# External object files for target alignment_prerejective
alignment_prerejective_EXTERNAL_OBJECTS =

alignment_prerejective: CMakeFiles/alignment_prerejective.dir/alignment_prerejective.cpp.o
alignment_prerejective: CMakeFiles/alignment_prerejective.dir/build.make
alignment_prerejective: /usr/lib/x86_64-linux-gnu/libboost_system.so
alignment_prerejective: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
alignment_prerejective: /usr/lib/x86_64-linux-gnu/libboost_thread.so
alignment_prerejective: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
alignment_prerejective: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
alignment_prerejective: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
alignment_prerejective: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
alignment_prerejective: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
alignment_prerejective: /usr/lib/x86_64-linux-gnu/libboost_regex.so
alignment_prerejective: /usr/local/lib/libpcl_common.so
alignment_prerejective: /usr/local/lib/libpcl_octree.so
alignment_prerejective: /usr/lib/libOpenNI.so
alignment_prerejective: /usr/local/lib/libpcl_io.so
alignment_prerejective: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
alignment_prerejective: /usr/local/lib/libpcl_kdtree.so
alignment_prerejective: /usr/local/lib/libpcl_search.so
alignment_prerejective: /usr/local/lib/libpcl_sample_consensus.so
alignment_prerejective: /usr/local/lib/libpcl_filters.so
alignment_prerejective: /usr/local/lib/libpcl_features.so
alignment_prerejective: /usr/local/lib/libpcl_registration.so
alignment_prerejective: /usr/local/lib/libpcl_ml.so
alignment_prerejective: /usr/local/lib/libpcl_segmentation.so
alignment_prerejective: /usr/local/lib/libpcl_visualization.so
alignment_prerejective: /usr/lib/x86_64-linux-gnu/libboost_system.so
alignment_prerejective: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
alignment_prerejective: /usr/lib/x86_64-linux-gnu/libboost_thread.so
alignment_prerejective: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
alignment_prerejective: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
alignment_prerejective: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
alignment_prerejective: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
alignment_prerejective: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
alignment_prerejective: /usr/lib/x86_64-linux-gnu/libboost_regex.so
alignment_prerejective: /usr/lib/libOpenNI.so
alignment_prerejective: /usr/lib/x86_64-linux-gnu/libflann_cpp.so
alignment_prerejective: /usr/lib/libvtkGenericFiltering.so.5.8.0
alignment_prerejective: /usr/lib/libvtkGeovis.so.5.8.0
alignment_prerejective: /usr/lib/libvtkCharts.so.5.8.0
alignment_prerejective: /usr/local/lib/libpcl_common.so
alignment_prerejective: /usr/local/lib/libpcl_octree.so
alignment_prerejective: /usr/local/lib/libpcl_io.so
alignment_prerejective: /usr/local/lib/libpcl_kdtree.so
alignment_prerejective: /usr/local/lib/libpcl_search.so
alignment_prerejective: /usr/local/lib/libpcl_sample_consensus.so
alignment_prerejective: /usr/local/lib/libpcl_filters.so
alignment_prerejective: /usr/local/lib/libpcl_features.so
alignment_prerejective: /usr/local/lib/libpcl_registration.so
alignment_prerejective: /usr/local/lib/libpcl_ml.so
alignment_prerejective: /usr/local/lib/libpcl_segmentation.so
alignment_prerejective: /usr/local/lib/libpcl_visualization.so
alignment_prerejective: /usr/lib/libvtkViews.so.5.8.0
alignment_prerejective: /usr/lib/libvtkInfovis.so.5.8.0
alignment_prerejective: /usr/lib/libvtkWidgets.so.5.8.0
alignment_prerejective: /usr/lib/libvtkVolumeRendering.so.5.8.0
alignment_prerejective: /usr/lib/libvtkHybrid.so.5.8.0
alignment_prerejective: /usr/lib/libvtkParallel.so.5.8.0
alignment_prerejective: /usr/lib/libvtkRendering.so.5.8.0
alignment_prerejective: /usr/lib/libvtkImaging.so.5.8.0
alignment_prerejective: /usr/lib/libvtkGraphics.so.5.8.0
alignment_prerejective: /usr/lib/libvtkIO.so.5.8.0
alignment_prerejective: /usr/lib/libvtkFiltering.so.5.8.0
alignment_prerejective: /usr/lib/libvtkCommon.so.5.8.0
alignment_prerejective: /usr/lib/libvtksys.so.5.8.0
alignment_prerejective: CMakeFiles/alignment_prerejective.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/damao/Documents/Script/pcl_test/pose_estimation/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable alignment_prerejective"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/alignment_prerejective.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/alignment_prerejective.dir/build: alignment_prerejective

.PHONY : CMakeFiles/alignment_prerejective.dir/build

CMakeFiles/alignment_prerejective.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/alignment_prerejective.dir/cmake_clean.cmake
.PHONY : CMakeFiles/alignment_prerejective.dir/clean

CMakeFiles/alignment_prerejective.dir/depend:
	cd /home/damao/Documents/Script/pcl_test/pose_estimation && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/damao/Documents/Script/pcl_test/pose_estimation /home/damao/Documents/Script/pcl_test/pose_estimation /home/damao/Documents/Script/pcl_test/pose_estimation /home/damao/Documents/Script/pcl_test/pose_estimation /home/damao/Documents/Script/pcl_test/pose_estimation/CMakeFiles/alignment_prerejective.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/alignment_prerejective.dir/depend

