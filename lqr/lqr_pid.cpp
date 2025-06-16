misys@AIX-411-UBUNTU-22:~/f1tenth_ws$ rm -rf log/ 
misys@AIX-411-UBUNTU-22:~/f1tenth_ws$ colcon build
Starting >>> asio_cmake_module
Starting >>> udp_msgs
Starting >>> vesc_msgs
Starting >>> teleop_tools_msgs
Starting >>> realsense2_camera_msgs
Starting >>> ackermann_mux
Starting >>> key_teleop
Starting >>> bringup
Starting >>> f1tenth_gym_ros
Starting >>> lqr_pid
Starting >>> mouse_teleop
Starting >>> mpc
Starting >>> particle_filter
Starting >>> pure_pursuit
Starting >>> safety_node
Starting >>> slam_toolbox
/usr/lib/python3/dist-packages/setuptools/dist.py:723: UserWarning: Usage of dash-separated 'script-dir' will not be supported in future versions. Please use the underscore name 'script_dir' instead
  warnings.warn(
/usr/lib/python3/dist-packages/setuptools/dist.py:723: UserWarning: Usage of dash-separated 'install-scripts' will not be supported in future versions. Please use the underscore name 'install_scripts' instead
  warnings.warn(
/usr/lib/python3/dist-packages/setuptools/dist.py:723: UserWarning: Usage of dash-separated 'script-dir' will not be supported in future versions. Please use the underscore name 'script_dir' instead
  warnings.warn(
/usr/lib/python3/dist-packages/setuptools/dist.py:723: UserWarning: Usage of dash-separated 'install-scripts' will not be supported in future versions. Please use the underscore name 'install_scripts' instead
  warnings.warn(
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
Failed   <<< lqr_pid [1.36s, exited with code 2]
Aborted  <<< key_teleop [1.38s]
Aborted  <<< bringup [1.38s]
Aborted  <<< f1tenth_gym_ros [1.37s]
Aborted  <<< mouse_teleop [1.36s]
Aborted  <<< particle_filter [1.35s]
Aborted  <<< ackermann_mux [1.41s]
Aborted  <<< asio_cmake_module [1.43s]
Aborted  <<< pure_pursuit [1.41s]
Aborted  <<< vesc_msgs [1.48s]
Aborted  <<< udp_msgs [1.50s]
Aborted  <<< teleop_tools_msgs [1.49s]
Aborted  <<< mpc [1.45s]
Aborted  <<< realsense2_camera_msgs [1.49s]                                               
Aborted  <<< safety_node [1.67s]                                                  
Aborted  <<< slam_toolbox [1.68s]

Summary: 0 packages finished [2.06s]
  1 package failed: lqr_pid
  15 packages aborted: ackermann_mux asio_cmake_module bringup f1tenth_gym_ros key_teleop mouse_teleop mpc particle_filter pure_pursuit realsense2_camera_msgs safety_node slam_toolbox teleop_tools_msgs udp_msgs vesc_msgs
  3 packages had stderr output: lqr_pid mpc safety_node
  12 packages not processed
