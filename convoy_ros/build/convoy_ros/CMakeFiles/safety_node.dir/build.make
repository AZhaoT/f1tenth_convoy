# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /sim_ws/src/convoy_ros

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /sim_ws/src/convoy_ros/build/convoy_ros

# Include any dependencies generated for this target.
include CMakeFiles/safety_node.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/safety_node.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/safety_node.dir/flags.make

CMakeFiles/safety_node.dir/src/safety.cpp.o: CMakeFiles/safety_node.dir/flags.make
CMakeFiles/safety_node.dir/src/safety.cpp.o: ../../src/safety.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/sim_ws/src/convoy_ros/build/convoy_ros/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/safety_node.dir/src/safety.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/safety_node.dir/src/safety.cpp.o -c /sim_ws/src/convoy_ros/src/safety.cpp

CMakeFiles/safety_node.dir/src/safety.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/safety_node.dir/src/safety.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /sim_ws/src/convoy_ros/src/safety.cpp > CMakeFiles/safety_node.dir/src/safety.cpp.i

CMakeFiles/safety_node.dir/src/safety.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/safety_node.dir/src/safety.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /sim_ws/src/convoy_ros/src/safety.cpp -o CMakeFiles/safety_node.dir/src/safety.cpp.s

CMakeFiles/safety_node.dir/src/safety_node.cpp.o: CMakeFiles/safety_node.dir/flags.make
CMakeFiles/safety_node.dir/src/safety_node.cpp.o: ../../src/safety_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/sim_ws/src/convoy_ros/build/convoy_ros/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/safety_node.dir/src/safety_node.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/safety_node.dir/src/safety_node.cpp.o -c /sim_ws/src/convoy_ros/src/safety_node.cpp

CMakeFiles/safety_node.dir/src/safety_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/safety_node.dir/src/safety_node.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /sim_ws/src/convoy_ros/src/safety_node.cpp > CMakeFiles/safety_node.dir/src/safety_node.cpp.i

CMakeFiles/safety_node.dir/src/safety_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/safety_node.dir/src/safety_node.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /sim_ws/src/convoy_ros/src/safety_node.cpp -o CMakeFiles/safety_node.dir/src/safety_node.cpp.s

# Object files for target safety_node
safety_node_OBJECTS = \
"CMakeFiles/safety_node.dir/src/safety.cpp.o" \
"CMakeFiles/safety_node.dir/src/safety_node.cpp.o"

# External object files for target safety_node
safety_node_EXTERNAL_OBJECTS =

safety_node: CMakeFiles/safety_node.dir/src/safety.cpp.o
safety_node: CMakeFiles/safety_node.dir/src/safety_node.cpp.o
safety_node: CMakeFiles/safety_node.dir/build.make
safety_node: /opt/ros/foxy/lib/librclcpp.so
safety_node: /opt/ros/foxy/lib/libackermann_msgs__rosidl_typesupport_introspection_c.so
safety_node: /opt/ros/foxy/lib/libackermann_msgs__rosidl_typesupport_c.so
safety_node: /opt/ros/foxy/lib/libackermann_msgs__rosidl_typesupport_introspection_cpp.so
safety_node: /opt/ros/foxy/lib/libackermann_msgs__rosidl_typesupport_cpp.so
safety_node: /opt/ros/foxy/lib/libnav_msgs__rosidl_typesupport_introspection_c.so
safety_node: /opt/ros/foxy/lib/libnav_msgs__rosidl_typesupport_c.so
safety_node: /opt/ros/foxy/lib/libnav_msgs__rosidl_typesupport_introspection_cpp.so
safety_node: /opt/ros/foxy/lib/libnav_msgs__rosidl_typesupport_cpp.so
safety_node: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
safety_node: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_c.so
safety_node: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
safety_node: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_cpp.so
safety_node: /opt/ros/foxy/lib/liblibstatistics_collector.so
safety_node: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_c.so
safety_node: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_generator_c.so
safety_node: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_c.so
safety_node: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_cpp.so
safety_node: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_cpp.so
safety_node: /opt/ros/foxy/lib/librcl.so
safety_node: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
safety_node: /opt/ros/foxy/lib/librcl_interfaces__rosidl_generator_c.so
safety_node: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_c.so
safety_node: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
safety_node: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_cpp.so
safety_node: /opt/ros/foxy/lib/librmw_implementation.so
safety_node: /opt/ros/foxy/lib/librmw.so
safety_node: /opt/ros/foxy/lib/librcl_logging_spdlog.so
safety_node: /usr/lib/x86_64-linux-gnu/libspdlog.so.1.5.0
safety_node: /opt/ros/foxy/lib/librcl_yaml_param_parser.so
safety_node: /opt/ros/foxy/lib/libyaml.so
safety_node: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
safety_node: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_generator_c.so
safety_node: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_c.so
safety_node: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
safety_node: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
safety_node: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
safety_node: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_generator_c.so
safety_node: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_c.so
safety_node: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
safety_node: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
safety_node: /opt/ros/foxy/lib/libtracetools.so
safety_node: /opt/ros/foxy/lib/libackermann_msgs__rosidl_generator_c.so
safety_node: /opt/ros/foxy/lib/libnav_msgs__rosidl_generator_c.so
safety_node: /opt/ros/foxy/lib/libsensor_msgs__rosidl_generator_c.so
safety_node: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
safety_node: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_generator_c.so
safety_node: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_c.so
safety_node: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
safety_node: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
safety_node: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
safety_node: /opt/ros/foxy/lib/libstd_msgs__rosidl_generator_c.so
safety_node: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_c.so
safety_node: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
safety_node: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_cpp.so
safety_node: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
safety_node: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
safety_node: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
safety_node: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
safety_node: /opt/ros/foxy/lib/librosidl_typesupport_introspection_cpp.so
safety_node: /opt/ros/foxy/lib/librosidl_typesupport_introspection_c.so
safety_node: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
safety_node: /opt/ros/foxy/lib/librosidl_typesupport_cpp.so
safety_node: /opt/ros/foxy/lib/librosidl_typesupport_c.so
safety_node: /opt/ros/foxy/lib/librcpputils.so
safety_node: /opt/ros/foxy/lib/librosidl_runtime_c.so
safety_node: /opt/ros/foxy/lib/librcutils.so
safety_node: CMakeFiles/safety_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/sim_ws/src/convoy_ros/build/convoy_ros/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable safety_node"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/safety_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/safety_node.dir/build: safety_node

.PHONY : CMakeFiles/safety_node.dir/build

CMakeFiles/safety_node.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/safety_node.dir/cmake_clean.cmake
.PHONY : CMakeFiles/safety_node.dir/clean

CMakeFiles/safety_node.dir/depend:
	cd /sim_ws/src/convoy_ros/build/convoy_ros && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /sim_ws/src/convoy_ros /sim_ws/src/convoy_ros /sim_ws/src/convoy_ros/build/convoy_ros /sim_ws/src/convoy_ros/build/convoy_ros /sim_ws/src/convoy_ros/build/convoy_ros/CMakeFiles/safety_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/safety_node.dir/depend

