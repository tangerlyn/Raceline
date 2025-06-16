-- stderr: f1tenth_gym_ros
/usr/lib/python3/dist-packages/setuptools/dist.py:723: UserWarning: Usage of dash-separated 'script-dir' will not be supported in future versions. Please use the underscore name 'script_dir' instead
  warnings.warn(
---
Finished <<< f1tenth_gym_ros [2.66s]
Finished <<< vesc [0.08s]                                                             
Finished <<< joy_teleop [1.26s]                                                   
Starting >>> teleop_tools
Starting >>> f1tenth_stack
/usr/lib/python3/dist-packages/setuptools/dist.py:723: UserWarning: Usage of dash-separated 'script-dir' will not be supported in future versions. Please use the underscore name 'script_dir' instead
  warnings.warn(
/usr/lib/python3/dist-packages/setuptools/dist.py:723: UserWarning: Usage of dash-separated 'install-scripts' will not be supported in future versions. Please use the underscore name 'install_scripts' instead
  warnings.warn(
Finished <<< teleop_tools [0.28s]                                                               
--- stderr: f1tenth_stack                                                            
/usr/lib/python3/dist-packages/setuptools/dist.py:723: UserWarning: Usage of dash-separated 'script-dir' will not be supported in future versions. Please use the underscore name 'script_dir' instead
  warnings.warn(
---
Finished <<< f1tenth_stack [0.92s]
--- stderr: lqr_pid                                  
/home/misys/f1tenth_ws/src/lqr_pid/src/lqr_pid_node.cpp: In member function ‘double LQRPID::pid_velocity(double, double, double, double)’:
/home/misys/f1tenth_ws/src/lqr_pid/src/lqr_pid_node.cpp:242:22: warning: unused variable ‘epsilon’ [-Wunused-variable]
  242 |     constexpr double epsilon = 1e-4;
      |                      ^~~~~~~
/usr/bin/ld: CMakeFiles/lqr_pid.dir/src/lqr_pid_node.cpp.o: in function `geometry_msgs::msg::PoseStamped_<std::allocator<void> >& tf2_ros::BufferInterface::transform<geometry_msgs::msg::PoseStamped_<std::allocator<void> > >(geometry_msgs::msg::PoseStamped_<std::allocator<void> > const&, geometry_msgs::msg::PoseStamped_<std::allocator<void> >&, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&, std::chrono::duration<long, std::ratio<1l, 1000000000l> >) const':
lqr_pid_node.cpp:(.text._ZNK7tf2_ros15BufferInterface9transformIN13geometry_msgs3msg12PoseStamped_ISaIvEEEEERT_RKS7_S8_RKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEENSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEE[_ZNK7tf2_ros15BufferInterface9transformIN13geometry_msgs3msg12PoseStamped_ISaIvEEEEERT_RKS7_S8_RKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEENSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEE]+0x5a): undefined reference to `std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::duration<long, std::ratio<1l, 1000000000l> > > tf2::getTimestamp<geometry_msgs::msg::PoseStamped_<std::allocator<void> > >(geometry_msgs::msg::PoseStamped_<std::allocator<void> > const&)'
/usr/bin/ld: lqr_pid_node.cpp:(.text._ZNK7tf2_ros15BufferInterface9transformIN13geometry_msgs3msg12PoseStamped_ISaIvEEEEERT_RKS7_S8_RKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEENSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEE[_ZNK7tf2_ros15BufferInterface9transformIN13geometry_msgs3msg12PoseStamped_ISaIvEEEEERT_RKS7_S8_RKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEENSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEE]+0x7a): undefined reference to `std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > tf2::getFrameId<geometry_msgs::msg::PoseStamped_<std::allocator<void> > >(geometry_msgs::msg::PoseStamped_<std::allocator<void> > const&)'
/usr/bin/ld: lqr_pid_node.cpp:(.text._ZNK7tf2_ros15BufferInterface9transformIN13geometry_msgs3msg12PoseStamped_ISaIvEEEEERT_RKS7_S8_RKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEENSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEE[_ZNK7tf2_ros15BufferInterface9transformIN13geometry_msgs3msg12PoseStamped_ISaIvEEEEERT_RKS7_S8_RKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEENSt6chrono8durationIlSt5ratioILl1ELl1000000000EEEE]+0xcf): undefined reference to `void tf2::doTransform<geometry_msgs::msg::PoseStamped_<std::allocator<void> > >(geometry_msgs::msg::PoseStamped_<std::allocator<void> > const&, geometry_msgs::msg::PoseStamped_<std::allocator<void> >&, geometry_msgs::msg::TransformStamped_<std::allocator<void> > const&)'
collect2: error: ld returned 1 exit status
gmake[2]: *** [CMakeFiles/lqr_pid.dir/build.make:230: lqr_pid] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:139: CMakeFiles/lqr_pid.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< lqr_pid [11.4s, exited with code 2]

Summary: 27 packages finished [11.7s]
  1 package failed: lqr_pid
  6 packages had stderr outpu
