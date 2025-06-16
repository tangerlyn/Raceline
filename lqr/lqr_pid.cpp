misys@AIX-411-UBUNTU-22:~/f1tenth_ws$ vim src/lqr_pid/CMakeLists.txt 
misys@AIX-411-UBUNTU-22:~/f1tenth_ws$ vim src/lqr_pid/CMakeLists.txt 
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
Finished <<< asio_cmake_module [1.55s]                                                              
--- stderr: lqr_pid
CMake Error at CMakeLists.txt:42 (target_link_libraries):
  Cannot specify link libraries for target "lqr_pid" which is not built by
  this project.


gmake: *** [Makefile:308: cmake_check_build_system] Error 1
---
Failed   <<< lqr_pid [1.51s, exited with code 2]
Aborted  <<< ackermann_mux [1.54s]
Aborted  <<< pure_pursuit [1.50s]
Aborted  <<< mpc [1.51s]
Aborted  <<< vesc_msgs [1.55s]
Aborted  <<< teleop_tools_msgs [1.55s]
Aborted  <<< udp_msgs [1.56s]
Aborted  <<< realsense2_camera_msgs [1.54s]
Aborted  <<< safety_node [1.74s]                                                      
Aborted  <<< f1tenth_gym_ros [1.82s]
Aborted  <<< particle_filter [1.82s]
Aborted  <<< bringup [1.86s]
Aborted  <<< mouse_teleop [1.85s]
Aborted  <<< slam_toolbox [1.83s]
Aborted  <<< key_teleop [1.91s]                       

Summary: 1 package finished [2.25s]
  1 package failed: lqr_pid
  14 packages aborted: ackermann_mux bringup f1tenth_gym_ros key_teleop mouse_teleop mpc particle_filter pure_pursuit realsense2_camera_msgs safety_node slam_toolbox teleop_tools_msgs udp_msgs vesc_msgs
  3 packages had stderr output: lqr_pid mpc safety_node
  12 packages not processed
