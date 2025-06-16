Starting >>> f1tenth_stack
/usr/lib/python3/dist-packages/setuptools/dist.py:723: UserWarning: Usage of dash-separated 'script-dir' will not be supported in future versions. Please use the underscore name 'script_dir' instead
  warnings.warn(
/usr/lib/python3/dist-packages/setuptools/dist.py:723: UserWarning: Usage of dash-separated 'install-scripts' will not be supported in future versions. Please use the underscore name 'install_scripts' instead
  warnings.warn(
Finished <<< teleop_tools [0.33s]                                                             
--- stderr: f1tenth_stack                                                            
/usr/lib/python3/dist-packages/setuptools/dist.py:723: UserWarning: Usage of dash-separated 'script-dir' will not be supported in future versions. Please use the underscore name 'script_dir' instead
  warnings.warn(
---
Finished <<< f1tenth_stack [0.92s]
--- stderr: lqr_pid                               
In file included from /home/misys/f1tenth_ws/src/lqr_pid/src/lqr_pid_node.cpp:1:
/home/misys/f1tenth_ws/src/lqr_pid/include/lqr_pid/lqr_pid.hpp:100:30: error: ‘Buffer’ is not a member of ‘tf2_ros’
  100 |     std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
      |                              ^~~~~~
/home/misys/f1tenth_ws/src/lqr_pid/include/lqr_pid/lqr_pid.hpp:100:36: error: template argument 1 is invalid
  100 |     std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
      |                                    ^
/home/misys/f1tenth_ws/src/lqr_pid/include/lqr_pid/lqr_pid.hpp:100:36: error: template argument 2 is invalid
/home/misys/f1tenth_ws/src/lqr_pid/src/lqr_pid_node.cpp: In constructor ‘LQRPID::LQRPID()’:
/home/misys/f1tenth_ws/src/lqr_pid/src/lqr_pid_node.cpp:107:47: error: cannot convert ‘std::_MakeUniq<tf2_ros::Buffer>::__single_object’ {aka ‘std::unique_ptr<tf2_ros::Buffer, std::default_delete<tf2_ros::Buffer> >’} to ‘int’ in assignment
  107 | tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      |              ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~^~~~~~~~~~~~~~~~~~~
      |                                               |
      |                                               std::_MakeUniq<tf2_ros::Buffer>::__single_object {aka std::unique_ptr<tf2_ros::Buffer, std::default_delete<tf2_ros::Buffer> >}
/home/misys/f1tenth_ws/src/lqr_pid/src/lqr_pid_node.cpp:108:72: error: invalid type argument of unary ‘*’ (have ‘int’)
  108 |     transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      |                                                                        ^~~~~~~~~~~
/home/misys/f1tenth_ws/src/lqr_pid/src/lqr_pid_node.cpp: In member function ‘double LQRPID::pid_velocity(double, double, double, double)’:
/home/misys/f1tenth_ws/src/lqr_pid/src/lqr_pid_node.cpp:242:22: warning: unused variable ‘epsilon’ [-Wunused-variable]
  242 |     constexpr double epsilon = 1e-4;
      |                      ^~~~~~~
/home/misys/f1tenth_ws/src/lqr_pid/src/lqr_pid_node.cpp: In member function ‘void LQRPID::odom_callback(nav_msgs::msg::Odometry_<std::allocator<void> >::ConstSharedPtr)’:
/home/misys/f1tenth_ws/src/lqr_pid/src/lqr_pid_node.cpp:347:19: error: base operand of ‘->’ is not a pointer
  347 |         tf_buffer_->transform(pose_in, pose_out, global_refFrame);  // 변환 대상: map
      |                   ^~
gmake[2]: *** [CMakeFiles/lqr_pid.dir/build.make:76: CMakeFiles/lqr_pid.dir/src/lqr_pid_node.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:139: CMakeFiles/lqr_pid.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< lqr_pid [6.91s, exited with code 2]

Summary: 27 packages finished [7.24s]
  1 package failed: lqr_pid
  6 packages had stderr output: f1tenth_gym_ros f1tenth_stack lqr_pid mpc particle_filter safety_node
misys@AIX-411-UBUNTU-22:~/f1tenth_ws$ 
