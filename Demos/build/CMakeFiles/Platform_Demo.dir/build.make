# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.25

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

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\CMake\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\CMake\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = C:\Users\johnw\Code\2D-Game-Framework\Demos

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = C:\Users\johnw\Code\2D-Game-Framework\Demos\build

# Include any dependencies generated for this target.
include CMakeFiles/Platform_Demo.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/Platform_Demo.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/Platform_Demo.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Platform_Demo.dir/flags.make

CMakeFiles/Platform_Demo.dir/Platform.cpp.obj: CMakeFiles/Platform_Demo.dir/flags.make
CMakeFiles/Platform_Demo.dir/Platform.cpp.obj: CMakeFiles/Platform_Demo.dir/includes_CXX.rsp
CMakeFiles/Platform_Demo.dir/Platform.cpp.obj: C:/Users/johnw/Code/2D-Game-Framework/Demos/Platform.cpp
CMakeFiles/Platform_Demo.dir/Platform.cpp.obj: CMakeFiles/Platform_Demo.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=C:\Users\johnw\Code\2D-Game-Framework\Demos\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Platform_Demo.dir/Platform.cpp.obj"
	C:\msys64\mingw64\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Platform_Demo.dir/Platform.cpp.obj -MF CMakeFiles\Platform_Demo.dir\Platform.cpp.obj.d -o CMakeFiles\Platform_Demo.dir\Platform.cpp.obj -c C:\Users\johnw\Code\2D-Game-Framework\Demos\Platform.cpp

CMakeFiles/Platform_Demo.dir/Platform.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Platform_Demo.dir/Platform.cpp.i"
	C:\msys64\mingw64\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E C:\Users\johnw\Code\2D-Game-Framework\Demos\Platform.cpp > CMakeFiles\Platform_Demo.dir\Platform.cpp.i

CMakeFiles/Platform_Demo.dir/Platform.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Platform_Demo.dir/Platform.cpp.s"
	C:\msys64\mingw64\bin\c++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S C:\Users\johnw\Code\2D-Game-Framework\Demos\Platform.cpp -o CMakeFiles\Platform_Demo.dir\Platform.cpp.s

# Object files for target Platform_Demo
Platform_Demo_OBJECTS = \
"CMakeFiles/Platform_Demo.dir/Platform.cpp.obj"

# External object files for target Platform_Demo
Platform_Demo_EXTERNAL_OBJECTS =

Platform_Demo.exe: CMakeFiles/Platform_Demo.dir/Platform.cpp.obj
Platform_Demo.exe: CMakeFiles/Platform_Demo.dir/build.make
Platform_Demo.exe: C:/Libraries/SDL2-2.26.2/x86_64-w64-mingw32/lib/libSDL2main.a
Platform_Demo.exe: C:/Libraries/SDL2-2.26.2/x86_64-w64-mingw32/lib/libSDL2.dll.a
Platform_Demo.exe: C:/Libraries/SDL2_image-2.6.2/x86_64-w64-mingw32/lib/libSDL2_image.dll.a
Platform_Demo.exe: C:/Libraries/SDL2_ttf-2.20.2/x86_64-w64-mingw32/lib/libSDL2_ttf.dll.a
Platform_Demo.exe: CMakeFiles/Platform_Demo.dir/linkLibs.rsp
Platform_Demo.exe: CMakeFiles/Platform_Demo.dir/objects1
Platform_Demo.exe: CMakeFiles/Platform_Demo.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=C:\Users\johnw\Code\2D-Game-Framework\Demos\build\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Platform_Demo.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\Platform_Demo.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Platform_Demo.dir/build: Platform_Demo.exe
.PHONY : CMakeFiles/Platform_Demo.dir/build

CMakeFiles/Platform_Demo.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\Platform_Demo.dir\cmake_clean.cmake
.PHONY : CMakeFiles/Platform_Demo.dir/clean

CMakeFiles/Platform_Demo.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" C:\Users\johnw\Code\2D-Game-Framework\Demos C:\Users\johnw\Code\2D-Game-Framework\Demos C:\Users\johnw\Code\2D-Game-Framework\Demos\build C:\Users\johnw\Code\2D-Game-Framework\Demos\build C:\Users\johnw\Code\2D-Game-Framework\Demos\build\CMakeFiles\Platform_Demo.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Platform_Demo.dir/depend
