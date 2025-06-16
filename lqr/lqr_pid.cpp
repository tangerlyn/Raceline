misys@AIX-411-UBUNTU-22:~/f1tenth_ws$ cat src/lqr_pid/CMakeLists.txt 
cmake_minimum_required(VERSION 3.8)
project(lqr_pid)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
foreach(PACKAGE
  ament_cmake
  rclcpp
  rclpy
  geometry_msgs
  nav_msgs
  ackermann_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  eigen3_cmake_module
  Eigen3
  tf2_ros
  tf2
  tf2_geometry_msgs
  )

  find_package(${PACKAGE} REQUIRED)

endforeach()

# Include Cpp "include" directory
include_directories(include)
include_directories(
  ${EIGEN3_INCLUDE_DIRS}
)



# Create Cpp executable
add_executable(lqr_pid src/lqr_pid_node.cpp)

ament_target_dependencies(lqr_pid
  rclcpp geometry_msgs ackermann_msgs nav_msgs sensor_msgs std_msgs visualization_msgs tf2_ros tf2 eigen3_cmake_module Eigen3 tf2_geometry_msgs
)



add_executable(waypoint_visualizer src/waypoint_visualizer.cpp)

ament_target_dependencies(waypoint_visualizer
  rclcpp geometry_msgs std_msgs visualization_msgs eigen3_cmake_module Eigen3
)

# Install Cpp executables
install(TARGETS
  lqr_pid
  waypoint_visualizer
  DESTINATION lib/${PROJECT_NAME})

# Install launch files and config
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

misys@AIX-411-UBUNTU-22:~/f1tenth_ws$ cat src/lqr_pid/package.xml    
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>lqr_pid</name>
  <version>0.0.0</version>
  <description>2025ADP project lqr_pid</description>
  <maintainer email="misys@todo.todo">misys</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>
  <buildtool_depend>eigen3_cmake_module</buildtool_depend>

  <build_depend>eigen</build_depend>

  <depend>rclcpp</depend>
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>sensor_msgs</depend>
  <depend>ackermann_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>visualization_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2</depend>
  <depend>Eigen3</depend>
  <depend>tf2_geometry_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>

  warnings.warn(
Finished <<< ackermann_mux [1.63s]                                                                  
Finished <<< asio_cmake_module [1.68s]
Starting >>> urg_node2
Finished <<< teleop_tools_msgs [1.74s]                                                      
Finished <<< vesc_msgs [1.78s]
Finished <<< pure_pursuit [1.75s]                                
Finished <<< realsense2_camera_msgs [1.84s]
Starting >>> joy_teleop
Starting >>> vesc_ackermann
Starting >>> realsense2_camera
Starting >>> realsense2_description
Finished <<< udp_msgs [1.95s]                                                        
Starting >>> io_context
--- stderr: mpc
/usr/lib/python3/dist-packages/setuptools/dist.py:723: UserWarning: Usage of dash-separated 'script-dir' will not be supported in future versions. Please use the underscore name 'script_dir' instead
  warnings.warn(
/usr/lib/python3/dist-packages/setuptools/dist.py:723: UserWarning: Usage of dash-separated 'install-scripts' will not be supported in future versions. Please use the underscore name 'install_scripts' instead
  warnings.warn(
---
Finished <<< mpc [1.95s]
Finished <<< urg_node2 [0.77s]                                              
--- stderr: safety_node
/usr/lib/python3/dist-packages/setuptools/dist.py:723: UserWarning: Usage of dash-separated 'script-dir' will not be supported in future versions. Please use the underscore name 'script_dir' instead
  warnings.warn(
/usr/lib/python3/dist-packages/setuptools/dist.py:723: UserWarning: Usage of dash-separated 'install-scripts' will not be supported in future versions. Please use the underscore name 'install_scripts' instead
  warnings.warn(
---
Finished <<< safety_node [2.44s]
Finished <<< realsense2_description [0.63s]                                  
--- stderr: lqr_pid
/usr/bin/ld: CMakeFiles/lqr_pid.dir/src/lqr_pid_node.cpp.o: in function `geometry_msgs::msg::PoseStamped_<std::allocator<void> >& tf2_ros::BufferInterface::transform<geometry_msgs::msg::PoseStamped_<std::allocator<void> > >(geometry_msgs::msg::PoseStamped_<std::allocator<void> > const&, geometry_msgs::msg::PoseStamped_<std::allocator<void> >&, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, std::chrono::duration<long, std::ratio<1l, 1000000000l> >) const':
lqr_pid_node.cpp:(.text._ZNK7tf2_ros15BufferInterface9transformIN13geometry_msgs3msg12PoseStamped_ISaIvEEEEERT_RKS7_S8_RKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEENSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEE[_ZNK7tf2_ros15BufferInterface9transformIN13geometry_msgs3msg12PoseStamped_ISaIvEEEEERT_RKS7_S8_RKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEENSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEE]+0x5a): undefined reference to `std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::duration<long, std::ratio<1l, 1000000000l> > > tf2::getTimestamp<geometry_msgs::msg::PoseStamped_<std::allocator<void> > >(geometry_msgs::msg::PoseStamped_<std::allocator<void> > const&)'
/usr/bin/ld: lqr_pid_node.cpp:(.text._ZNK7tf2_ros15BufferInterface9transformIN13geometry_msgs3msg12PoseStamped_ISaIvEEEEERT_RKS7_S8_RKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEENSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEE[_ZNK7tf2_ros15BufferInterface9transformIN13geometry_msgs3msg12PoseStamped_ISaIvEEEEERT_RKS7_S8_RKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEENSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEE]+0x7a): undefined reference to `std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > tf2::getFrameId<geometry_msgs::msg::PoseStamped_<std::allocator<void> > >(geometry_msgs::msg::PoseStamped_<std::allocator<void> > const&)'
/usr/bin/ld: lqr_pid_node.cpp:(.text._ZNK7tf2_ros15BufferInterface9transformIN13geometry_msgs3msg12PoseStamped_ISaIvEEEEERT_RKS7_S8_RKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEENSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEE[_ZNK7tf2_ros15BufferInterface9transformIN13geometry_msgs3msg12PoseStamped_ISaIvEEEEERT_RKS7_S8_RKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEENSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEE]+0xcf): undefined reference to `void tf2::doTransform<geometry_msgs::msg::PoseStamped_<std::allocator<void> > >(geometry_msgs::msg::PoseStamped_<std::allocator<void> > const&, geometry_msgs::msg::PoseStamped_<std::allocator<void> >&, geometry_msgs::msg::TransformStamped_<std::allocator<void> > const&)'
collect2: error: ld returned 1 exit status
gmake[2]: *** [CMakeFiles/lqr_pid.dir/build.make:230: lqr_pid] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:139: CMakeFiles/lqr_pid.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< lqr_pid [2.50s, exited with code 2]
Finished <<< slam_toolbox [2.50s]
Aborted  <<< realsense2_camera [0.69s]
Aborted  <<< io_context [0.61s]
Aborted  <<< vesc_ackermann [0.70s]
Aborted  <<< mouse_teleop [2.67s]                                                      
Aborted  <<< f1tenth_gym_ros [2.70s]
Aborted  <<< key_teleop [2.72s]
Aborted  <<< particle_filter [2.73s]
Aborted  <<< bringup [2.76s]
Aborted  <<< joy_teleop [0.95s]
                                  
Summary: 12 packages finished [3.11s]
  1 package failed: lqr_pid
  9 packages aborted: bringup f1tenth_gym_ros io_context joy_teleop key_teleop mouse_teleop particle_filter realsense2_camera vesc_ackermann
  5 packages had stderr output: f1tenth_gym_ros lqr_pid mpc particle_filter safety_node
  6 packages not processed

