Finished <<< f1tenth_stack [0.88s]
--- stderr: lqr_pid                                  
/usr/bin/ld: CMakeFiles/lqr_pid.dir/src/lqr_pid_node.cpp.o: in function `LQRPID::odom_callback(std::shared_ptr<nav_msgs::msg::Odometry_<std::allocator<void> > const>)':
lqr_pid_node.cpp:(.text+0x5b08): undefined reference to `void tf2::fromMsg<geometry_msgs::msg::Transform_<std::allocator<void> >, tf2::Transform>(geometry_msgs::msg::Transform_<std::allocator<void> > const&, tf2::Transform&)'
/usr/bin/ld: lqr_pid_node.cpp:(.text+0x5ba1): undefined reference to `void tf2::fromMsg<geometry_msgs::msg::Quaternion_<std::allocator<void> >, tf2::Quaternion>(geometry_msgs::msg::Quaternion_<std::allocator<void> > const&, tf2::Quaternion&)'
collect2: error: ld returned 1 exit status
gmake[2]: *** [CMakeFiles/lqr_pid.dir/build.make:229: lqr_pid] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:139: CMakeFiles/lqr_pid.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
