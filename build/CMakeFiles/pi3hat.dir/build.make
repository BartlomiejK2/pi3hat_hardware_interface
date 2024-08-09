# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/bartek/KNR/meldog-ros/src/meldog_hardware/pi3hat_hardware_interface

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bartek/KNR/meldog-ros/src/meldog_hardware/pi3hat_hardware_interface/build

# Include any dependencies generated for this target.
include CMakeFiles/pi3hat.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/pi3hat.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/pi3hat.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/pi3hat.dir/flags.make

CMakeFiles/pi3hat.dir/include/3rd_libs/pi3hat/pi3hat.cc.o: CMakeFiles/pi3hat.dir/flags.make
CMakeFiles/pi3hat.dir/include/3rd_libs/pi3hat/pi3hat.cc.o: ../include/3rd_libs/pi3hat/pi3hat.cc
CMakeFiles/pi3hat.dir/include/3rd_libs/pi3hat/pi3hat.cc.o: CMakeFiles/pi3hat.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bartek/KNR/meldog-ros/src/meldog_hardware/pi3hat_hardware_interface/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/pi3hat.dir/include/3rd_libs/pi3hat/pi3hat.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/pi3hat.dir/include/3rd_libs/pi3hat/pi3hat.cc.o -MF CMakeFiles/pi3hat.dir/include/3rd_libs/pi3hat/pi3hat.cc.o.d -o CMakeFiles/pi3hat.dir/include/3rd_libs/pi3hat/pi3hat.cc.o -c /home/bartek/KNR/meldog-ros/src/meldog_hardware/pi3hat_hardware_interface/include/3rd_libs/pi3hat/pi3hat.cc

CMakeFiles/pi3hat.dir/include/3rd_libs/pi3hat/pi3hat.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/pi3hat.dir/include/3rd_libs/pi3hat/pi3hat.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bartek/KNR/meldog-ros/src/meldog_hardware/pi3hat_hardware_interface/include/3rd_libs/pi3hat/pi3hat.cc > CMakeFiles/pi3hat.dir/include/3rd_libs/pi3hat/pi3hat.cc.i

CMakeFiles/pi3hat.dir/include/3rd_libs/pi3hat/pi3hat.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/pi3hat.dir/include/3rd_libs/pi3hat/pi3hat.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bartek/KNR/meldog-ros/src/meldog_hardware/pi3hat_hardware_interface/include/3rd_libs/pi3hat/pi3hat.cc -o CMakeFiles/pi3hat.dir/include/3rd_libs/pi3hat/pi3hat.cc.s

# Object files for target pi3hat
pi3hat_OBJECTS = \
"CMakeFiles/pi3hat.dir/include/3rd_libs/pi3hat/pi3hat.cc.o"

# External object files for target pi3hat
pi3hat_EXTERNAL_OBJECTS =

libpi3hat.so: CMakeFiles/pi3hat.dir/include/3rd_libs/pi3hat/pi3hat.cc.o
libpi3hat.so: CMakeFiles/pi3hat.dir/build.make
libpi3hat.so: CMakeFiles/pi3hat.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bartek/KNR/meldog-ros/src/meldog_hardware/pi3hat_hardware_interface/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libpi3hat.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/pi3hat.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/pi3hat.dir/build: libpi3hat.so
.PHONY : CMakeFiles/pi3hat.dir/build

CMakeFiles/pi3hat.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/pi3hat.dir/cmake_clean.cmake
.PHONY : CMakeFiles/pi3hat.dir/clean

CMakeFiles/pi3hat.dir/depend:
	cd /home/bartek/KNR/meldog-ros/src/meldog_hardware/pi3hat_hardware_interface/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bartek/KNR/meldog-ros/src/meldog_hardware/pi3hat_hardware_interface /home/bartek/KNR/meldog-ros/src/meldog_hardware/pi3hat_hardware_interface /home/bartek/KNR/meldog-ros/src/meldog_hardware/pi3hat_hardware_interface/build /home/bartek/KNR/meldog-ros/src/meldog_hardware/pi3hat_hardware_interface/build /home/bartek/KNR/meldog-ros/src/meldog_hardware/pi3hat_hardware_interface/build/CMakeFiles/pi3hat.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/pi3hat.dir/depend
