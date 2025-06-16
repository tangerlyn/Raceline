/home/misys/f1tenth_ws/src/lqr_pid/include/lqr_pid/lqr_pid.hpp:100:30: error: ‘Buffer’ is not a member of ‘tf2_ros’
  100 |     std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
      |                              ^~~~~~
/home/misys/f1tenth_ws/src/lqr_pid/include/lqr_pid/lqr_pid.hpp:100:36: error: template argument 1 is invalid
  100 |     std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
      |                                    ^
/home/misys/f1tenth_ws/src/lqr_pid/include/lqr_pid/lqr_pid.hpp:100:36: error: template argument 2 is invalid
/home/misys/f1tenth_ws/src/lqr_pid/src/lqr_pid_node.cpp: In member function ‘double LQRPID::pid_velocity(double, double, double, double)’:
/home/misys/f1tenth_ws/src/lqr_pid/src/lqr_pid_node.cpp:241:22: warning: unused variable ‘epsilon’ [-Wunused-variable]
  241 |     constexpr double epsilon = 1e-4;
      |                      ^~~~~~~
gmake[2]: *** [CMakeFiles/lqr_pid.dir/build.make:76: CMakeFiles/lqr_pid.dir/src/lqr_pid_node.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:139: CMakeFiles/lqr_pid.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< lqr_pid [7.17s, exited with code 2]

Summary: 27 packages finished [7.50s]
