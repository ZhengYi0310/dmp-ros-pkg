# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib

# Include any dependencies generated for this target.
include CMakeFiles/lwr_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/lwr_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lwr_test.dir/flags.make

CMakeFiles/lwr_test.dir/test_lwr/test_locally_weigthed_regression.cpp.o: CMakeFiles/lwr_test.dir/flags.make
CMakeFiles/lwr_test.dir/test_lwr/test_locally_weigthed_regression.cpp.o: test_lwr/test_locally_weigthed_regression.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/lwr_test.dir/test_lwr/test_locally_weigthed_regression.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/lwr_test.dir/test_lwr/test_locally_weigthed_regression.cpp.o -c /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/test_lwr/test_locally_weigthed_regression.cpp

CMakeFiles/lwr_test.dir/test_lwr/test_locally_weigthed_regression.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lwr_test.dir/test_lwr/test_locally_weigthed_regression.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/test_lwr/test_locally_weigthed_regression.cpp > CMakeFiles/lwr_test.dir/test_lwr/test_locally_weigthed_regression.cpp.i

CMakeFiles/lwr_test.dir/test_lwr/test_locally_weigthed_regression.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lwr_test.dir/test_lwr/test_locally_weigthed_regression.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/test_lwr/test_locally_weigthed_regression.cpp -o CMakeFiles/lwr_test.dir/test_lwr/test_locally_weigthed_regression.cpp.s

CMakeFiles/lwr_test.dir/test_lwr/test_locally_weigthed_regression.cpp.o.requires:
.PHONY : CMakeFiles/lwr_test.dir/test_lwr/test_locally_weigthed_regression.cpp.o.requires

CMakeFiles/lwr_test.dir/test_lwr/test_locally_weigthed_regression.cpp.o.provides: CMakeFiles/lwr_test.dir/test_lwr/test_locally_weigthed_regression.cpp.o.requires
	$(MAKE) -f CMakeFiles/lwr_test.dir/build.make CMakeFiles/lwr_test.dir/test_lwr/test_locally_weigthed_regression.cpp.o.provides.build
.PHONY : CMakeFiles/lwr_test.dir/test_lwr/test_locally_weigthed_regression.cpp.o.provides

CMakeFiles/lwr_test.dir/test_lwr/test_locally_weigthed_regression.cpp.o.provides.build: CMakeFiles/lwr_test.dir/test_lwr/test_locally_weigthed_regression.cpp.o

# Object files for target lwr_test
lwr_test_OBJECTS = \
"CMakeFiles/lwr_test.dir/test_lwr/test_locally_weigthed_regression.cpp.o"

# External object files for target lwr_test
lwr_test_EXTERNAL_OBJECTS =

devel/lib/dynamic_movement_primitive/lwr_test: CMakeFiles/lwr_test.dir/test_lwr/test_locally_weigthed_regression.cpp.o
devel/lib/dynamic_movement_primitive/lwr_test: CMakeFiles/lwr_test.dir/build.make
devel/lib/dynamic_movement_primitive/lwr_test: devel/lib/liblwr.so
devel/lib/dynamic_movement_primitive/lwr_test: CMakeFiles/lwr_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable devel/lib/dynamic_movement_primitive/lwr_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lwr_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lwr_test.dir/build: devel/lib/dynamic_movement_primitive/lwr_test
.PHONY : CMakeFiles/lwr_test.dir/build

CMakeFiles/lwr_test.dir/requires: CMakeFiles/lwr_test.dir/test_lwr/test_locally_weigthed_regression.cpp.o.requires
.PHONY : CMakeFiles/lwr_test.dir/requires

CMakeFiles/lwr_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lwr_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lwr_test.dir/clean

CMakeFiles/lwr_test.dir/depend:
	cd /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib /home/yzheng/yzheng_ws/src/dmp/dynamic_movement_primitive/dmpLib/CMakeFiles/lwr_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/lwr_test.dir/depend

