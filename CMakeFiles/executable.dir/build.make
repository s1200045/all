# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.3

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
CMAKE_COMMAND = /opt/local/bin/cmake

# The command to remove a file.
RM = /opt/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/s1200045/all

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/s1200045/all

# Include any dependencies generated for this target.
include CMakeFiles/executable.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/executable.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/executable.dir/flags.make

CMakeFiles/executable.dir/Camera.cpp.o: CMakeFiles/executable.dir/flags.make
CMakeFiles/executable.dir/Camera.cpp.o: Camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/s1200045/all/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/executable.dir/Camera.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/executable.dir/Camera.cpp.o -c /Users/s1200045/all/Camera.cpp

CMakeFiles/executable.dir/Camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/executable.dir/Camera.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/s1200045/all/Camera.cpp > CMakeFiles/executable.dir/Camera.cpp.i

CMakeFiles/executable.dir/Camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/executable.dir/Camera.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/s1200045/all/Camera.cpp -o CMakeFiles/executable.dir/Camera.cpp.s

CMakeFiles/executable.dir/Camera.cpp.o.requires:

.PHONY : CMakeFiles/executable.dir/Camera.cpp.o.requires

CMakeFiles/executable.dir/Camera.cpp.o.provides: CMakeFiles/executable.dir/Camera.cpp.o.requires
	$(MAKE) -f CMakeFiles/executable.dir/build.make CMakeFiles/executable.dir/Camera.cpp.o.provides.build
.PHONY : CMakeFiles/executable.dir/Camera.cpp.o.provides

CMakeFiles/executable.dir/Camera.cpp.o.provides.build: CMakeFiles/executable.dir/Camera.cpp.o


CMakeFiles/executable.dir/Image.cpp.o: CMakeFiles/executable.dir/flags.make
CMakeFiles/executable.dir/Image.cpp.o: Image.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/s1200045/all/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/executable.dir/Image.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/executable.dir/Image.cpp.o -c /Users/s1200045/all/Image.cpp

CMakeFiles/executable.dir/Image.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/executable.dir/Image.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/s1200045/all/Image.cpp > CMakeFiles/executable.dir/Image.cpp.i

CMakeFiles/executable.dir/Image.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/executable.dir/Image.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/s1200045/all/Image.cpp -o CMakeFiles/executable.dir/Image.cpp.s

CMakeFiles/executable.dir/Image.cpp.o.requires:

.PHONY : CMakeFiles/executable.dir/Image.cpp.o.requires

CMakeFiles/executable.dir/Image.cpp.o.provides: CMakeFiles/executable.dir/Image.cpp.o.requires
	$(MAKE) -f CMakeFiles/executable.dir/build.make CMakeFiles/executable.dir/Image.cpp.o.provides.build
.PHONY : CMakeFiles/executable.dir/Image.cpp.o.provides

CMakeFiles/executable.dir/Image.cpp.o.provides.build: CMakeFiles/executable.dir/Image.cpp.o


CMakeFiles/executable.dir/Shader.cpp.o: CMakeFiles/executable.dir/flags.make
CMakeFiles/executable.dir/Shader.cpp.o: Shader.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/s1200045/all/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/executable.dir/Shader.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/executable.dir/Shader.cpp.o -c /Users/s1200045/all/Shader.cpp

CMakeFiles/executable.dir/Shader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/executable.dir/Shader.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/s1200045/all/Shader.cpp > CMakeFiles/executable.dir/Shader.cpp.i

CMakeFiles/executable.dir/Shader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/executable.dir/Shader.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/s1200045/all/Shader.cpp -o CMakeFiles/executable.dir/Shader.cpp.s

CMakeFiles/executable.dir/Shader.cpp.o.requires:

.PHONY : CMakeFiles/executable.dir/Shader.cpp.o.requires

CMakeFiles/executable.dir/Shader.cpp.o.provides: CMakeFiles/executable.dir/Shader.cpp.o.requires
	$(MAKE) -f CMakeFiles/executable.dir/build.make CMakeFiles/executable.dir/Shader.cpp.o.provides.build
.PHONY : CMakeFiles/executable.dir/Shader.cpp.o.provides

CMakeFiles/executable.dir/Shader.cpp.o.provides.build: CMakeFiles/executable.dir/Shader.cpp.o


CMakeFiles/executable.dir/TriMesh.cpp.o: CMakeFiles/executable.dir/flags.make
CMakeFiles/executable.dir/TriMesh.cpp.o: TriMesh.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/s1200045/all/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/executable.dir/TriMesh.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/executable.dir/TriMesh.cpp.o -c /Users/s1200045/all/TriMesh.cpp

CMakeFiles/executable.dir/TriMesh.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/executable.dir/TriMesh.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/s1200045/all/TriMesh.cpp > CMakeFiles/executable.dir/TriMesh.cpp.i

CMakeFiles/executable.dir/TriMesh.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/executable.dir/TriMesh.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/s1200045/all/TriMesh.cpp -o CMakeFiles/executable.dir/TriMesh.cpp.s

CMakeFiles/executable.dir/TriMesh.cpp.o.requires:

.PHONY : CMakeFiles/executable.dir/TriMesh.cpp.o.requires

CMakeFiles/executable.dir/TriMesh.cpp.o.provides: CMakeFiles/executable.dir/TriMesh.cpp.o.requires
	$(MAKE) -f CMakeFiles/executable.dir/build.make CMakeFiles/executable.dir/TriMesh.cpp.o.provides.build
.PHONY : CMakeFiles/executable.dir/TriMesh.cpp.o.provides

CMakeFiles/executable.dir/TriMesh.cpp.o.provides.build: CMakeFiles/executable.dir/TriMesh.cpp.o


CMakeFiles/executable.dir/Viewer.cpp.o: CMakeFiles/executable.dir/flags.make
CMakeFiles/executable.dir/Viewer.cpp.o: Viewer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/s1200045/all/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/executable.dir/Viewer.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/executable.dir/Viewer.cpp.o -c /Users/s1200045/all/Viewer.cpp

CMakeFiles/executable.dir/Viewer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/executable.dir/Viewer.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/s1200045/all/Viewer.cpp > CMakeFiles/executable.dir/Viewer.cpp.i

CMakeFiles/executable.dir/Viewer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/executable.dir/Viewer.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/s1200045/all/Viewer.cpp -o CMakeFiles/executable.dir/Viewer.cpp.s

CMakeFiles/executable.dir/Viewer.cpp.o.requires:

.PHONY : CMakeFiles/executable.dir/Viewer.cpp.o.requires

CMakeFiles/executable.dir/Viewer.cpp.o.provides: CMakeFiles/executable.dir/Viewer.cpp.o.requires
	$(MAKE) -f CMakeFiles/executable.dir/build.make CMakeFiles/executable.dir/Viewer.cpp.o.provides.build
.PHONY : CMakeFiles/executable.dir/Viewer.cpp.o.provides

CMakeFiles/executable.dir/Viewer.cpp.o.provides.build: CMakeFiles/executable.dir/Viewer.cpp.o


CMakeFiles/executable.dir/main.cpp.o: CMakeFiles/executable.dir/flags.make
CMakeFiles/executable.dir/main.cpp.o: main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/s1200045/all/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/executable.dir/main.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/executable.dir/main.cpp.o -c /Users/s1200045/all/main.cpp

CMakeFiles/executable.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/executable.dir/main.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/s1200045/all/main.cpp > CMakeFiles/executable.dir/main.cpp.i

CMakeFiles/executable.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/executable.dir/main.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/s1200045/all/main.cpp -o CMakeFiles/executable.dir/main.cpp.s

CMakeFiles/executable.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/executable.dir/main.cpp.o.requires

CMakeFiles/executable.dir/main.cpp.o.provides: CMakeFiles/executable.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/executable.dir/build.make CMakeFiles/executable.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/executable.dir/main.cpp.o.provides

CMakeFiles/executable.dir/main.cpp.o.provides.build: CMakeFiles/executable.dir/main.cpp.o


CMakeFiles/executable.dir/types.cpp.o: CMakeFiles/executable.dir/flags.make
CMakeFiles/executable.dir/types.cpp.o: types.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/s1200045/all/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/executable.dir/types.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/executable.dir/types.cpp.o -c /Users/s1200045/all/types.cpp

CMakeFiles/executable.dir/types.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/executable.dir/types.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/s1200045/all/types.cpp > CMakeFiles/executable.dir/types.cpp.i

CMakeFiles/executable.dir/types.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/executable.dir/types.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/s1200045/all/types.cpp -o CMakeFiles/executable.dir/types.cpp.s

CMakeFiles/executable.dir/types.cpp.o.requires:

.PHONY : CMakeFiles/executable.dir/types.cpp.o.requires

CMakeFiles/executable.dir/types.cpp.o.provides: CMakeFiles/executable.dir/types.cpp.o.requires
	$(MAKE) -f CMakeFiles/executable.dir/build.make CMakeFiles/executable.dir/types.cpp.o.provides.build
.PHONY : CMakeFiles/executable.dir/types.cpp.o.provides

CMakeFiles/executable.dir/types.cpp.o.provides.build: CMakeFiles/executable.dir/types.cpp.o


# Object files for target executable
executable_OBJECTS = \
"CMakeFiles/executable.dir/Camera.cpp.o" \
"CMakeFiles/executable.dir/Image.cpp.o" \
"CMakeFiles/executable.dir/Shader.cpp.o" \
"CMakeFiles/executable.dir/TriMesh.cpp.o" \
"CMakeFiles/executable.dir/Viewer.cpp.o" \
"CMakeFiles/executable.dir/main.cpp.o" \
"CMakeFiles/executable.dir/types.cpp.o"

# External object files for target executable
executable_EXTERNAL_OBJECTS =

executable: CMakeFiles/executable.dir/Camera.cpp.o
executable: CMakeFiles/executable.dir/Image.cpp.o
executable: CMakeFiles/executable.dir/Shader.cpp.o
executable: CMakeFiles/executable.dir/TriMesh.cpp.o
executable: CMakeFiles/executable.dir/Viewer.cpp.o
executable: CMakeFiles/executable.dir/main.cpp.o
executable: CMakeFiles/executable.dir/types.cpp.o
executable: CMakeFiles/executable.dir/build.make
executable: /opt/local/lib/libmpfr.dylib
executable: /opt/local/lib/libgmp.dylib
executable: /opt/local/lib/libCGAL.dylib
executable: /opt/local/lib/libboost_thread-mt.dylib
executable: /opt/local/lib/libboost_system-mt.dylib
executable: /opt/local/lib/libCGAL.dylib
executable: /opt/local/lib/libboost_thread-mt.dylib
executable: /opt/local/lib/libboost_system-mt.dylib
executable: CMakeFiles/executable.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/s1200045/all/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX executable executable"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/executable.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/executable.dir/build: executable

.PHONY : CMakeFiles/executable.dir/build

CMakeFiles/executable.dir/requires: CMakeFiles/executable.dir/Camera.cpp.o.requires
CMakeFiles/executable.dir/requires: CMakeFiles/executable.dir/Image.cpp.o.requires
CMakeFiles/executable.dir/requires: CMakeFiles/executable.dir/Shader.cpp.o.requires
CMakeFiles/executable.dir/requires: CMakeFiles/executable.dir/TriMesh.cpp.o.requires
CMakeFiles/executable.dir/requires: CMakeFiles/executable.dir/Viewer.cpp.o.requires
CMakeFiles/executable.dir/requires: CMakeFiles/executable.dir/main.cpp.o.requires
CMakeFiles/executable.dir/requires: CMakeFiles/executable.dir/types.cpp.o.requires

.PHONY : CMakeFiles/executable.dir/requires

CMakeFiles/executable.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/executable.dir/cmake_clean.cmake
.PHONY : CMakeFiles/executable.dir/clean

CMakeFiles/executable.dir/depend:
	cd /Users/s1200045/all && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/s1200045/all /Users/s1200045/all /Users/s1200045/all /Users/s1200045/all /Users/s1200045/all/CMakeFiles/executable.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/executable.dir/depend

