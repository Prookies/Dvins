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
CMAKE_SOURCE_DIR = /home/lab202/Code/Test/Dvins/Dvins_lib

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lab202/Code/Test/Dvins/Dvins_lib/build

# Include any dependencies generated for this target.
include CMakeFiles/Dvins.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Dvins.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Dvins.dir/flags.make

CMakeFiles/Dvins.dir/src/System.cc.o: CMakeFiles/Dvins.dir/flags.make
CMakeFiles/Dvins.dir/src/System.cc.o: ../src/System.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lab202/Code/Test/Dvins/Dvins_lib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Dvins.dir/src/System.cc.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Dvins.dir/src/System.cc.o -c /home/lab202/Code/Test/Dvins/Dvins_lib/src/System.cc

CMakeFiles/Dvins.dir/src/System.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Dvins.dir/src/System.cc.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lab202/Code/Test/Dvins/Dvins_lib/src/System.cc > CMakeFiles/Dvins.dir/src/System.cc.i

CMakeFiles/Dvins.dir/src/System.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Dvins.dir/src/System.cc.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lab202/Code/Test/Dvins/Dvins_lib/src/System.cc -o CMakeFiles/Dvins.dir/src/System.cc.s

CMakeFiles/Dvins.dir/src/System.cc.o.requires:

.PHONY : CMakeFiles/Dvins.dir/src/System.cc.o.requires

CMakeFiles/Dvins.dir/src/System.cc.o.provides: CMakeFiles/Dvins.dir/src/System.cc.o.requires
	$(MAKE) -f CMakeFiles/Dvins.dir/build.make CMakeFiles/Dvins.dir/src/System.cc.o.provides.build
.PHONY : CMakeFiles/Dvins.dir/src/System.cc.o.provides

CMakeFiles/Dvins.dir/src/System.cc.o.provides.build: CMakeFiles/Dvins.dir/src/System.cc.o


CMakeFiles/Dvins.dir/src/Tracking.cc.o: CMakeFiles/Dvins.dir/flags.make
CMakeFiles/Dvins.dir/src/Tracking.cc.o: ../src/Tracking.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lab202/Code/Test/Dvins/Dvins_lib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Dvins.dir/src/Tracking.cc.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Dvins.dir/src/Tracking.cc.o -c /home/lab202/Code/Test/Dvins/Dvins_lib/src/Tracking.cc

CMakeFiles/Dvins.dir/src/Tracking.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Dvins.dir/src/Tracking.cc.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lab202/Code/Test/Dvins/Dvins_lib/src/Tracking.cc > CMakeFiles/Dvins.dir/src/Tracking.cc.i

CMakeFiles/Dvins.dir/src/Tracking.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Dvins.dir/src/Tracking.cc.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lab202/Code/Test/Dvins/Dvins_lib/src/Tracking.cc -o CMakeFiles/Dvins.dir/src/Tracking.cc.s

CMakeFiles/Dvins.dir/src/Tracking.cc.o.requires:

.PHONY : CMakeFiles/Dvins.dir/src/Tracking.cc.o.requires

CMakeFiles/Dvins.dir/src/Tracking.cc.o.provides: CMakeFiles/Dvins.dir/src/Tracking.cc.o.requires
	$(MAKE) -f CMakeFiles/Dvins.dir/build.make CMakeFiles/Dvins.dir/src/Tracking.cc.o.provides.build
.PHONY : CMakeFiles/Dvins.dir/src/Tracking.cc.o.provides

CMakeFiles/Dvins.dir/src/Tracking.cc.o.provides.build: CMakeFiles/Dvins.dir/src/Tracking.cc.o


CMakeFiles/Dvins.dir/src/Optimizer.cc.o: CMakeFiles/Dvins.dir/flags.make
CMakeFiles/Dvins.dir/src/Optimizer.cc.o: ../src/Optimizer.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lab202/Code/Test/Dvins/Dvins_lib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Dvins.dir/src/Optimizer.cc.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Dvins.dir/src/Optimizer.cc.o -c /home/lab202/Code/Test/Dvins/Dvins_lib/src/Optimizer.cc

CMakeFiles/Dvins.dir/src/Optimizer.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Dvins.dir/src/Optimizer.cc.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lab202/Code/Test/Dvins/Dvins_lib/src/Optimizer.cc > CMakeFiles/Dvins.dir/src/Optimizer.cc.i

CMakeFiles/Dvins.dir/src/Optimizer.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Dvins.dir/src/Optimizer.cc.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lab202/Code/Test/Dvins/Dvins_lib/src/Optimizer.cc -o CMakeFiles/Dvins.dir/src/Optimizer.cc.s

CMakeFiles/Dvins.dir/src/Optimizer.cc.o.requires:

.PHONY : CMakeFiles/Dvins.dir/src/Optimizer.cc.o.requires

CMakeFiles/Dvins.dir/src/Optimizer.cc.o.provides: CMakeFiles/Dvins.dir/src/Optimizer.cc.o.requires
	$(MAKE) -f CMakeFiles/Dvins.dir/build.make CMakeFiles/Dvins.dir/src/Optimizer.cc.o.provides.build
.PHONY : CMakeFiles/Dvins.dir/src/Optimizer.cc.o.provides

CMakeFiles/Dvins.dir/src/Optimizer.cc.o.provides.build: CMakeFiles/Dvins.dir/src/Optimizer.cc.o


CMakeFiles/Dvins.dir/src/KeyFrame.cc.o: CMakeFiles/Dvins.dir/flags.make
CMakeFiles/Dvins.dir/src/KeyFrame.cc.o: ../src/KeyFrame.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lab202/Code/Test/Dvins/Dvins_lib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/Dvins.dir/src/KeyFrame.cc.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Dvins.dir/src/KeyFrame.cc.o -c /home/lab202/Code/Test/Dvins/Dvins_lib/src/KeyFrame.cc

CMakeFiles/Dvins.dir/src/KeyFrame.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Dvins.dir/src/KeyFrame.cc.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lab202/Code/Test/Dvins/Dvins_lib/src/KeyFrame.cc > CMakeFiles/Dvins.dir/src/KeyFrame.cc.i

CMakeFiles/Dvins.dir/src/KeyFrame.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Dvins.dir/src/KeyFrame.cc.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lab202/Code/Test/Dvins/Dvins_lib/src/KeyFrame.cc -o CMakeFiles/Dvins.dir/src/KeyFrame.cc.s

CMakeFiles/Dvins.dir/src/KeyFrame.cc.o.requires:

.PHONY : CMakeFiles/Dvins.dir/src/KeyFrame.cc.o.requires

CMakeFiles/Dvins.dir/src/KeyFrame.cc.o.provides: CMakeFiles/Dvins.dir/src/KeyFrame.cc.o.requires
	$(MAKE) -f CMakeFiles/Dvins.dir/build.make CMakeFiles/Dvins.dir/src/KeyFrame.cc.o.provides.build
.PHONY : CMakeFiles/Dvins.dir/src/KeyFrame.cc.o.provides

CMakeFiles/Dvins.dir/src/KeyFrame.cc.o.provides.build: CMakeFiles/Dvins.dir/src/KeyFrame.cc.o


CMakeFiles/Dvins.dir/src/Frame.cc.o: CMakeFiles/Dvins.dir/flags.make
CMakeFiles/Dvins.dir/src/Frame.cc.o: ../src/Frame.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lab202/Code/Test/Dvins/Dvins_lib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/Dvins.dir/src/Frame.cc.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Dvins.dir/src/Frame.cc.o -c /home/lab202/Code/Test/Dvins/Dvins_lib/src/Frame.cc

CMakeFiles/Dvins.dir/src/Frame.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Dvins.dir/src/Frame.cc.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lab202/Code/Test/Dvins/Dvins_lib/src/Frame.cc > CMakeFiles/Dvins.dir/src/Frame.cc.i

CMakeFiles/Dvins.dir/src/Frame.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Dvins.dir/src/Frame.cc.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lab202/Code/Test/Dvins/Dvins_lib/src/Frame.cc -o CMakeFiles/Dvins.dir/src/Frame.cc.s

CMakeFiles/Dvins.dir/src/Frame.cc.o.requires:

.PHONY : CMakeFiles/Dvins.dir/src/Frame.cc.o.requires

CMakeFiles/Dvins.dir/src/Frame.cc.o.provides: CMakeFiles/Dvins.dir/src/Frame.cc.o.requires
	$(MAKE) -f CMakeFiles/Dvins.dir/build.make CMakeFiles/Dvins.dir/src/Frame.cc.o.provides.build
.PHONY : CMakeFiles/Dvins.dir/src/Frame.cc.o.provides

CMakeFiles/Dvins.dir/src/Frame.cc.o.provides.build: CMakeFiles/Dvins.dir/src/Frame.cc.o


CMakeFiles/Dvins.dir/src/Initializer.cc.o: CMakeFiles/Dvins.dir/flags.make
CMakeFiles/Dvins.dir/src/Initializer.cc.o: ../src/Initializer.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lab202/Code/Test/Dvins/Dvins_lib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/Dvins.dir/src/Initializer.cc.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Dvins.dir/src/Initializer.cc.o -c /home/lab202/Code/Test/Dvins/Dvins_lib/src/Initializer.cc

CMakeFiles/Dvins.dir/src/Initializer.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Dvins.dir/src/Initializer.cc.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lab202/Code/Test/Dvins/Dvins_lib/src/Initializer.cc > CMakeFiles/Dvins.dir/src/Initializer.cc.i

CMakeFiles/Dvins.dir/src/Initializer.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Dvins.dir/src/Initializer.cc.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lab202/Code/Test/Dvins/Dvins_lib/src/Initializer.cc -o CMakeFiles/Dvins.dir/src/Initializer.cc.s

CMakeFiles/Dvins.dir/src/Initializer.cc.o.requires:

.PHONY : CMakeFiles/Dvins.dir/src/Initializer.cc.o.requires

CMakeFiles/Dvins.dir/src/Initializer.cc.o.provides: CMakeFiles/Dvins.dir/src/Initializer.cc.o.requires
	$(MAKE) -f CMakeFiles/Dvins.dir/build.make CMakeFiles/Dvins.dir/src/Initializer.cc.o.provides.build
.PHONY : CMakeFiles/Dvins.dir/src/Initializer.cc.o.provides

CMakeFiles/Dvins.dir/src/Initializer.cc.o.provides.build: CMakeFiles/Dvins.dir/src/Initializer.cc.o


CMakeFiles/Dvins.dir/src/ImuData.cc.o: CMakeFiles/Dvins.dir/flags.make
CMakeFiles/Dvins.dir/src/ImuData.cc.o: ../src/ImuData.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lab202/Code/Test/Dvins/Dvins_lib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/Dvins.dir/src/ImuData.cc.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Dvins.dir/src/ImuData.cc.o -c /home/lab202/Code/Test/Dvins/Dvins_lib/src/ImuData.cc

CMakeFiles/Dvins.dir/src/ImuData.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Dvins.dir/src/ImuData.cc.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lab202/Code/Test/Dvins/Dvins_lib/src/ImuData.cc > CMakeFiles/Dvins.dir/src/ImuData.cc.i

CMakeFiles/Dvins.dir/src/ImuData.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Dvins.dir/src/ImuData.cc.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lab202/Code/Test/Dvins/Dvins_lib/src/ImuData.cc -o CMakeFiles/Dvins.dir/src/ImuData.cc.s

CMakeFiles/Dvins.dir/src/ImuData.cc.o.requires:

.PHONY : CMakeFiles/Dvins.dir/src/ImuData.cc.o.requires

CMakeFiles/Dvins.dir/src/ImuData.cc.o.provides: CMakeFiles/Dvins.dir/src/ImuData.cc.o.requires
	$(MAKE) -f CMakeFiles/Dvins.dir/build.make CMakeFiles/Dvins.dir/src/ImuData.cc.o.provides.build
.PHONY : CMakeFiles/Dvins.dir/src/ImuData.cc.o.provides

CMakeFiles/Dvins.dir/src/ImuData.cc.o.provides.build: CMakeFiles/Dvins.dir/src/ImuData.cc.o


CMakeFiles/Dvins.dir/src/ConfigParam.cc.o: CMakeFiles/Dvins.dir/flags.make
CMakeFiles/Dvins.dir/src/ConfigParam.cc.o: ../src/ConfigParam.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lab202/Code/Test/Dvins/Dvins_lib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/Dvins.dir/src/ConfigParam.cc.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Dvins.dir/src/ConfigParam.cc.o -c /home/lab202/Code/Test/Dvins/Dvins_lib/src/ConfigParam.cc

CMakeFiles/Dvins.dir/src/ConfigParam.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Dvins.dir/src/ConfigParam.cc.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lab202/Code/Test/Dvins/Dvins_lib/src/ConfigParam.cc > CMakeFiles/Dvins.dir/src/ConfigParam.cc.i

CMakeFiles/Dvins.dir/src/ConfigParam.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Dvins.dir/src/ConfigParam.cc.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lab202/Code/Test/Dvins/Dvins_lib/src/ConfigParam.cc -o CMakeFiles/Dvins.dir/src/ConfigParam.cc.s

CMakeFiles/Dvins.dir/src/ConfigParam.cc.o.requires:

.PHONY : CMakeFiles/Dvins.dir/src/ConfigParam.cc.o.requires

CMakeFiles/Dvins.dir/src/ConfigParam.cc.o.provides: CMakeFiles/Dvins.dir/src/ConfigParam.cc.o.requires
	$(MAKE) -f CMakeFiles/Dvins.dir/build.make CMakeFiles/Dvins.dir/src/ConfigParam.cc.o.provides.build
.PHONY : CMakeFiles/Dvins.dir/src/ConfigParam.cc.o.provides

CMakeFiles/Dvins.dir/src/ConfigParam.cc.o.provides.build: CMakeFiles/Dvins.dir/src/ConfigParam.cc.o


CMakeFiles/Dvins.dir/src/FeatureTracker.cc.o: CMakeFiles/Dvins.dir/flags.make
CMakeFiles/Dvins.dir/src/FeatureTracker.cc.o: ../src/FeatureTracker.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lab202/Code/Test/Dvins/Dvins_lib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/Dvins.dir/src/FeatureTracker.cc.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Dvins.dir/src/FeatureTracker.cc.o -c /home/lab202/Code/Test/Dvins/Dvins_lib/src/FeatureTracker.cc

CMakeFiles/Dvins.dir/src/FeatureTracker.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Dvins.dir/src/FeatureTracker.cc.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lab202/Code/Test/Dvins/Dvins_lib/src/FeatureTracker.cc > CMakeFiles/Dvins.dir/src/FeatureTracker.cc.i

CMakeFiles/Dvins.dir/src/FeatureTracker.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Dvins.dir/src/FeatureTracker.cc.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lab202/Code/Test/Dvins/Dvins_lib/src/FeatureTracker.cc -o CMakeFiles/Dvins.dir/src/FeatureTracker.cc.s

CMakeFiles/Dvins.dir/src/FeatureTracker.cc.o.requires:

.PHONY : CMakeFiles/Dvins.dir/src/FeatureTracker.cc.o.requires

CMakeFiles/Dvins.dir/src/FeatureTracker.cc.o.provides: CMakeFiles/Dvins.dir/src/FeatureTracker.cc.o.requires
	$(MAKE) -f CMakeFiles/Dvins.dir/build.make CMakeFiles/Dvins.dir/src/FeatureTracker.cc.o.provides.build
.PHONY : CMakeFiles/Dvins.dir/src/FeatureTracker.cc.o.provides

CMakeFiles/Dvins.dir/src/FeatureTracker.cc.o.provides.build: CMakeFiles/Dvins.dir/src/FeatureTracker.cc.o


# Object files for target Dvins
Dvins_OBJECTS = \
"CMakeFiles/Dvins.dir/src/System.cc.o" \
"CMakeFiles/Dvins.dir/src/Tracking.cc.o" \
"CMakeFiles/Dvins.dir/src/Optimizer.cc.o" \
"CMakeFiles/Dvins.dir/src/KeyFrame.cc.o" \
"CMakeFiles/Dvins.dir/src/Frame.cc.o" \
"CMakeFiles/Dvins.dir/src/Initializer.cc.o" \
"CMakeFiles/Dvins.dir/src/ImuData.cc.o" \
"CMakeFiles/Dvins.dir/src/ConfigParam.cc.o" \
"CMakeFiles/Dvins.dir/src/FeatureTracker.cc.o"

# External object files for target Dvins
Dvins_EXTERNAL_OBJECTS =

../lib/libDvins.so: CMakeFiles/Dvins.dir/src/System.cc.o
../lib/libDvins.so: CMakeFiles/Dvins.dir/src/Tracking.cc.o
../lib/libDvins.so: CMakeFiles/Dvins.dir/src/Optimizer.cc.o
../lib/libDvins.so: CMakeFiles/Dvins.dir/src/KeyFrame.cc.o
../lib/libDvins.so: CMakeFiles/Dvins.dir/src/Frame.cc.o
../lib/libDvins.so: CMakeFiles/Dvins.dir/src/Initializer.cc.o
../lib/libDvins.so: CMakeFiles/Dvins.dir/src/ImuData.cc.o
../lib/libDvins.so: CMakeFiles/Dvins.dir/src/ConfigParam.cc.o
../lib/libDvins.so: CMakeFiles/Dvins.dir/src/FeatureTracker.cc.o
../lib/libDvins.so: CMakeFiles/Dvins.dir/build.make
../lib/libDvins.so: /usr/local/lib/libopencv_cudabgsegm.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_cudaobjdetect.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_cudastereo.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_stitching.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_superres.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_videostab.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_cudaoptflow.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_xobjdetect.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_surface_matching.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_dpm.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_hfs.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_optflow.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_dnn_objdetect.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_stereo.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_sfm.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_reg.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_freetype.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_xphoto.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_img_hash.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_structured_light.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_saliency.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_hdf.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_ximgproc.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_face.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_bgsegm.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_ccalib.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_tracking.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_cvv.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_line_descriptor.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_datasets.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_fuzzy.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_bioinspired.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_aruco.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_rgbd.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libceres.a
../lib/libDvins.so: /usr/local/lib/libopencv_cudafeatures2d.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_cudacodec.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_cudawarping.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_cudalegacy.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_xfeatures2d.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_shape.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_viz.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_phase_unwrapping.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_objdetect.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_photo.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_cudaimgproc.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_cudafilters.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_cudaarithm.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_plot.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_text.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_ml.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_dnn.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_video.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_calib3d.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_features2d.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_highgui.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_flann.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_videoio.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_imgcodecs.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_imgproc.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_core.so.3.4.1
../lib/libDvins.so: /usr/local/lib/libopencv_cudev.so.3.4.1
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libglog.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libgflags.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libspqr.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libtbb.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libtbbmalloc.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libcholmod.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libccolamd.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libcamd.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libcolamd.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libamd.so
../lib/libDvins.so: /usr/lib/liblapack.so
../lib/libDvins.so: /usr/lib/libf77blas.so
../lib/libDvins.so: /usr/lib/libatlas.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/librt.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libcxsparse.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libspqr.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libtbb.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libtbbmalloc.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libcholmod.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libccolamd.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libcamd.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libcolamd.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libamd.so
../lib/libDvins.so: /usr/lib/liblapack.so
../lib/libDvins.so: /usr/lib/libf77blas.so
../lib/libDvins.so: /usr/lib/libatlas.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/librt.so
../lib/libDvins.so: /usr/lib/x86_64-linux-gnu/libcxsparse.so
../lib/libDvins.so: CMakeFiles/Dvins.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lab202/Code/Test/Dvins/Dvins_lib/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking CXX shared library ../lib/libDvins.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Dvins.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Dvins.dir/build: ../lib/libDvins.so

.PHONY : CMakeFiles/Dvins.dir/build

CMakeFiles/Dvins.dir/requires: CMakeFiles/Dvins.dir/src/System.cc.o.requires
CMakeFiles/Dvins.dir/requires: CMakeFiles/Dvins.dir/src/Tracking.cc.o.requires
CMakeFiles/Dvins.dir/requires: CMakeFiles/Dvins.dir/src/Optimizer.cc.o.requires
CMakeFiles/Dvins.dir/requires: CMakeFiles/Dvins.dir/src/KeyFrame.cc.o.requires
CMakeFiles/Dvins.dir/requires: CMakeFiles/Dvins.dir/src/Frame.cc.o.requires
CMakeFiles/Dvins.dir/requires: CMakeFiles/Dvins.dir/src/Initializer.cc.o.requires
CMakeFiles/Dvins.dir/requires: CMakeFiles/Dvins.dir/src/ImuData.cc.o.requires
CMakeFiles/Dvins.dir/requires: CMakeFiles/Dvins.dir/src/ConfigParam.cc.o.requires
CMakeFiles/Dvins.dir/requires: CMakeFiles/Dvins.dir/src/FeatureTracker.cc.o.requires

.PHONY : CMakeFiles/Dvins.dir/requires

CMakeFiles/Dvins.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Dvins.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Dvins.dir/clean

CMakeFiles/Dvins.dir/depend:
	cd /home/lab202/Code/Test/Dvins/Dvins_lib/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lab202/Code/Test/Dvins/Dvins_lib /home/lab202/Code/Test/Dvins/Dvins_lib /home/lab202/Code/Test/Dvins/Dvins_lib/build /home/lab202/Code/Test/Dvins/Dvins_lib/build /home/lab202/Code/Test/Dvins/Dvins_lib/build/CMakeFiles/Dvins.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Dvins.dir/depend

