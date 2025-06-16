Finished <<< teleop_tools [0.34s]                                                                 
--- stderr: f1tenth_stack                                                                         
/usr/lib/python3/dist-packages/setuptools/dist.py:723: UserWarning: Usage of dash-separated 'script-dir' will not be supported in future versions. Please use the underscore name 'script_dir' instead
  warnings.warn(
---
Finished <<< f1tenth_stack [1.05s]
--- stderr: lqr_pid                                                                           
/home/misys/f1tenth_ws/src/lqr_pid/src/lqr_pid_node.cpp: In member function ‘void LQRPID::odom_callback(nav_msgs::msg::Odometry_<std::allocator<void> >::ConstSharedPtr)’:
/home/misys/f1tenth_ws/src/lqr_pid/src/lqr_pid_node.cpp:349:21: error: no matching function for call to ‘tf2::Matrix3x3::Matrix3x3(Eigen::Quaterniond&)’
  349 |     tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      |                     ^
In file included from /opt/ros/humble/include/tf2/tf2/LinearMath/Transform.hpp:21,
                 from /opt/ros/humble/include/tf2/tf2/buffer_core.hpp:47,
                 from /opt/ros/humble/include/tf2_ros/tf2_ros/transform_listener.h:40,
                 from /home/misys/f1tenth_ws/src/lqr_pid/include/lqr_pid/lqr_pid.hpp:6,
                 from /home/misys/f1tenth_ws/src/lqr_pid/src/lqr_pid_node.cpp:1:
/opt/ros/humble/include/tf2/tf2/LinearMath/Matrix3x3.hpp:66:30: note: candidate: ‘tf2::Matrix3x3::Matrix3x3(const tf2::Matrix3x3&)’
   66 |         TF2SIMD_FORCE_INLINE Matrix3x3 (const Matrix3x3& other)
      |                              ^~~~~~~~~
/opt/ros/humble/include/tf2/tf2/LinearMath/Matrix3x3.hpp:66:58: note:   no known conversion for argument 1 from ‘Eigen::Quaterniond’ {aka ‘Eigen::Quaternion<double>’} to ‘const tf2::Matrix3x3&’
   66 |         TF2SIMD_FORCE_INLINE Matrix3x3 (const Matrix3x3& other)
      |                                         ~~~~~~~~~~~~~~~~~^~~~~
/opt/ros/humble/include/tf2/tf2/LinearMath/Matrix3x3.hpp:57:9: note: candidate: ‘tf2::Matrix3x3::Matrix3x3(const tf2Scalar&, const tf2Scalar&, const tf2Scalar&, const tf2Scalar&, const tf2Scalar&, const tf2Scalar&, const tf2Scalar&, const tf2Scalar&, const tf2Scalar&)’
   57 |         Matrix3x3(const tf2Scalar& xx, const tf2Scalar& xy, const tf2Scalar& xz,
      |         ^~~~~~~~~
/opt/ros/humble/include/tf2/tf2/LinearMath/Matrix3x3.hpp:57:9: note:   candidate expects 9 arguments, 1 provided
/opt/ros/humble/include/tf2/tf2/LinearMath/Matrix3x3.hpp:47:18: note: candidate: ‘tf2::Matrix3x3::Matrix3x3(const tf2::Quaternion&)’
   47 |         explicit Matrix3x3(const Quaternion& q) { setRotation(q); }
      |                  ^~~~~~~~~
/opt/ros/humble/include/tf2/tf2/LinearMath/Matrix3x3.hpp:47:46: note:   no known conversion for argument 1 from ‘Eigen::Quaterniond’ {aka ‘Eigen::Quaternion<double>’} to ‘const tf2::Quaternion&’
   47 |         explicit Matrix3x3(const Quaternion& q) { setRotation(q); }
      |                            ~~~~~~~~~~~~~~~~~~^
/opt/ros/humble/include/tf2/tf2/LinearMath/Matrix3x3.hpp:41:9: note: candidate: ‘tf2::Matrix3x3::Matrix3x3()’
   41 |         Matrix3x3 () {}
      |         ^~~~~~~~~
/opt/ros/humble/include/tf2/tf2/LinearMath/Matrix3x3.hpp:41:9: note:   candidate expects 0 arguments, 1 provided
gmake[2]: *** [CMakeFiles/lqr_pid.dir/build.make:76: CMakeFiles/lqr_pid.dir/src/lqr_pid_node.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:139: CMakeFiles/lqr_pid.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< lqr_pid [7.91s, exited with code 2]
Aborted  <<< pure_pursuit [11.8s]                                    

Summary: 26 packages finished [12.2s]
  1 package failed: lqr_pid
  1 package aborted: pure_pursuit
  7 packages had stderr output: f1tenth_gym_ros f1tenth_stack lqr_pid mpc particle_filter pure_pursuit safety_node
