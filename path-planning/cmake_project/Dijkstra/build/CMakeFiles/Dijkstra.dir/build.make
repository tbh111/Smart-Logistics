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
CMAKE_SOURCE_DIR = /home/tbh/Desktop/Dijkstra

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tbh/Desktop/Dijkstra/build

# Include any dependencies generated for this target.
include CMakeFiles/Dijkstra.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Dijkstra.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Dijkstra.dir/flags.make

CMakeFiles/Dijkstra.dir/src/main.cpp.o: CMakeFiles/Dijkstra.dir/flags.make
CMakeFiles/Dijkstra.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tbh/Desktop/Dijkstra/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Dijkstra.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Dijkstra.dir/src/main.cpp.o -c /home/tbh/Desktop/Dijkstra/src/main.cpp

CMakeFiles/Dijkstra.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Dijkstra.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tbh/Desktop/Dijkstra/src/main.cpp > CMakeFiles/Dijkstra.dir/src/main.cpp.i

CMakeFiles/Dijkstra.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Dijkstra.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tbh/Desktop/Dijkstra/src/main.cpp -o CMakeFiles/Dijkstra.dir/src/main.cpp.s

CMakeFiles/Dijkstra.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/Dijkstra.dir/src/main.cpp.o.requires

CMakeFiles/Dijkstra.dir/src/main.cpp.o.provides: CMakeFiles/Dijkstra.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/Dijkstra.dir/build.make CMakeFiles/Dijkstra.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/Dijkstra.dir/src/main.cpp.o.provides

CMakeFiles/Dijkstra.dir/src/main.cpp.o.provides.build: CMakeFiles/Dijkstra.dir/src/main.cpp.o


CMakeFiles/Dijkstra.dir/src/dijkstra.cpp.o: CMakeFiles/Dijkstra.dir/flags.make
CMakeFiles/Dijkstra.dir/src/dijkstra.cpp.o: ../src/dijkstra.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tbh/Desktop/Dijkstra/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Dijkstra.dir/src/dijkstra.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Dijkstra.dir/src/dijkstra.cpp.o -c /home/tbh/Desktop/Dijkstra/src/dijkstra.cpp

CMakeFiles/Dijkstra.dir/src/dijkstra.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Dijkstra.dir/src/dijkstra.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tbh/Desktop/Dijkstra/src/dijkstra.cpp > CMakeFiles/Dijkstra.dir/src/dijkstra.cpp.i

CMakeFiles/Dijkstra.dir/src/dijkstra.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Dijkstra.dir/src/dijkstra.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tbh/Desktop/Dijkstra/src/dijkstra.cpp -o CMakeFiles/Dijkstra.dir/src/dijkstra.cpp.s

CMakeFiles/Dijkstra.dir/src/dijkstra.cpp.o.requires:

.PHONY : CMakeFiles/Dijkstra.dir/src/dijkstra.cpp.o.requires

CMakeFiles/Dijkstra.dir/src/dijkstra.cpp.o.provides: CMakeFiles/Dijkstra.dir/src/dijkstra.cpp.o.requires
	$(MAKE) -f CMakeFiles/Dijkstra.dir/build.make CMakeFiles/Dijkstra.dir/src/dijkstra.cpp.o.provides.build
.PHONY : CMakeFiles/Dijkstra.dir/src/dijkstra.cpp.o.provides

CMakeFiles/Dijkstra.dir/src/dijkstra.cpp.o.provides.build: CMakeFiles/Dijkstra.dir/src/dijkstra.cpp.o


# Object files for target Dijkstra
Dijkstra_OBJECTS = \
"CMakeFiles/Dijkstra.dir/src/main.cpp.o" \
"CMakeFiles/Dijkstra.dir/src/dijkstra.cpp.o"

# External object files for target Dijkstra
Dijkstra_EXTERNAL_OBJECTS =

../bin/Dijkstra: CMakeFiles/Dijkstra.dir/src/main.cpp.o
../bin/Dijkstra: CMakeFiles/Dijkstra.dir/src/dijkstra.cpp.o
../bin/Dijkstra: CMakeFiles/Dijkstra.dir/build.make
../bin/Dijkstra: CMakeFiles/Dijkstra.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tbh/Desktop/Dijkstra/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ../bin/Dijkstra"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Dijkstra.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Dijkstra.dir/build: ../bin/Dijkstra

.PHONY : CMakeFiles/Dijkstra.dir/build

CMakeFiles/Dijkstra.dir/requires: CMakeFiles/Dijkstra.dir/src/main.cpp.o.requires
CMakeFiles/Dijkstra.dir/requires: CMakeFiles/Dijkstra.dir/src/dijkstra.cpp.o.requires

.PHONY : CMakeFiles/Dijkstra.dir/requires

CMakeFiles/Dijkstra.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Dijkstra.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Dijkstra.dir/clean

CMakeFiles/Dijkstra.dir/depend:
	cd /home/tbh/Desktop/Dijkstra/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tbh/Desktop/Dijkstra /home/tbh/Desktop/Dijkstra /home/tbh/Desktop/Dijkstra/build /home/tbh/Desktop/Dijkstra/build /home/tbh/Desktop/Dijkstra/build/CMakeFiles/Dijkstra.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Dijkstra.dir/depend

