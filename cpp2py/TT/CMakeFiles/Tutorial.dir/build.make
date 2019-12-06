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
CMAKE_SOURCE_DIR = /home/nvidia/autoparking/cpp2py/TT

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/nvidia/autoparking/cpp2py/TT

# Include any dependencies generated for this target.
include CMakeFiles/Tutorial.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Tutorial.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Tutorial.dir/flags.make

CMakeFiles/Tutorial.dir/tt.cpp.o: CMakeFiles/Tutorial.dir/flags.make
CMakeFiles/Tutorial.dir/tt.cpp.o: tt.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/nvidia/autoparking/cpp2py/TT/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Tutorial.dir/tt.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Tutorial.dir/tt.cpp.o -c /home/nvidia/autoparking/cpp2py/TT/tt.cpp

CMakeFiles/Tutorial.dir/tt.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Tutorial.dir/tt.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/nvidia/autoparking/cpp2py/TT/tt.cpp > CMakeFiles/Tutorial.dir/tt.cpp.i

CMakeFiles/Tutorial.dir/tt.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Tutorial.dir/tt.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/nvidia/autoparking/cpp2py/TT/tt.cpp -o CMakeFiles/Tutorial.dir/tt.cpp.s

CMakeFiles/Tutorial.dir/tt.cpp.o.requires:

.PHONY : CMakeFiles/Tutorial.dir/tt.cpp.o.requires

CMakeFiles/Tutorial.dir/tt.cpp.o.provides: CMakeFiles/Tutorial.dir/tt.cpp.o.requires
	$(MAKE) -f CMakeFiles/Tutorial.dir/build.make CMakeFiles/Tutorial.dir/tt.cpp.o.provides.build
.PHONY : CMakeFiles/Tutorial.dir/tt.cpp.o.provides

CMakeFiles/Tutorial.dir/tt.cpp.o.provides.build: CMakeFiles/Tutorial.dir/tt.cpp.o


# Object files for target Tutorial
Tutorial_OBJECTS = \
"CMakeFiles/Tutorial.dir/tt.cpp.o"

# External object files for target Tutorial
Tutorial_EXTERNAL_OBJECTS =

Tutorial: CMakeFiles/Tutorial.dir/tt.cpp.o
Tutorial: CMakeFiles/Tutorial.dir/build.make
Tutorial: /usr/local/lib/libopencv_cudabgsegm.so.3.4.3
Tutorial: /usr/local/lib/libopencv_cudaobjdetect.so.3.4.3
Tutorial: /usr/local/lib/libopencv_cudastereo.so.3.4.3
Tutorial: /usr/local/lib/libopencv_dnn.so.3.4.3
Tutorial: /usr/local/lib/libopencv_ml.so.3.4.3
Tutorial: /usr/local/lib/libopencv_shape.so.3.4.3
Tutorial: /usr/local/lib/libopencv_stitching.so.3.4.3
Tutorial: /usr/local/lib/libopencv_superres.so.3.4.3
Tutorial: /usr/local/lib/libopencv_videostab.so.3.4.3
Tutorial: /usr/local/lib/libopencv_viz.so.3.4.3
Tutorial: /usr/local/lib/libopencv_cudafeatures2d.so.3.4.3
Tutorial: /usr/local/lib/libopencv_cudacodec.so.3.4.3
Tutorial: /usr/local/lib/libopencv_cudaoptflow.so.3.4.3
Tutorial: /usr/local/lib/libopencv_cudalegacy.so.3.4.3
Tutorial: /usr/local/lib/libopencv_calib3d.so.3.4.3
Tutorial: /usr/local/lib/libopencv_cudawarping.so.3.4.3
Tutorial: /usr/local/lib/libopencv_features2d.so.3.4.3
Tutorial: /usr/local/lib/libopencv_flann.so.3.4.3
Tutorial: /usr/local/lib/libopencv_highgui.so.3.4.3
Tutorial: /usr/local/lib/libopencv_objdetect.so.3.4.3
Tutorial: /usr/local/lib/libopencv_photo.so.3.4.3
Tutorial: /usr/local/lib/libopencv_cudaimgproc.so.3.4.3
Tutorial: /usr/local/lib/libopencv_cudafilters.so.3.4.3
Tutorial: /usr/local/lib/libopencv_cudaarithm.so.3.4.3
Tutorial: /usr/local/lib/libopencv_video.so.3.4.3
Tutorial: /usr/local/lib/libopencv_videoio.so.3.4.3
Tutorial: /usr/local/lib/libopencv_imgcodecs.so.3.4.3
Tutorial: /usr/local/lib/libopencv_imgproc.so.3.4.3
Tutorial: /usr/local/lib/libopencv_core.so.3.4.3
Tutorial: /usr/local/lib/libopencv_cudev.so.3.4.3
Tutorial: CMakeFiles/Tutorial.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/nvidia/autoparking/cpp2py/TT/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Tutorial"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Tutorial.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Tutorial.dir/build: Tutorial

.PHONY : CMakeFiles/Tutorial.dir/build

CMakeFiles/Tutorial.dir/requires: CMakeFiles/Tutorial.dir/tt.cpp.o.requires

.PHONY : CMakeFiles/Tutorial.dir/requires

CMakeFiles/Tutorial.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Tutorial.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Tutorial.dir/clean

CMakeFiles/Tutorial.dir/depend:
	cd /home/nvidia/autoparking/cpp2py/TT && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/nvidia/autoparking/cpp2py/TT /home/nvidia/autoparking/cpp2py/TT /home/nvidia/autoparking/cpp2py/TT /home/nvidia/autoparking/cpp2py/TT /home/nvidia/autoparking/cpp2py/TT/CMakeFiles/Tutorial.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Tutorial.dir/depend
