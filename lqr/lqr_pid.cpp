Starting >>> urg_node2
Finished <<< ackermann_mux [1.51s]
Finished <<< realsense2_camera_msgs [1.56s]
Finished <<< teleop_tools_msgs [1.60s]                                                      
Finished <<< vesc_msgs [1.63s]
Finished <<< udp_msgs [1.67s]
Starting >>> realsense2_camera
Starting >>> realsense2_description
Finished <<< pure_pursuit [1.66s]                                           
Starting >>> io_context
Starting >>> joy_teleop
Starting >>> vesc_ackermann
--- stderr: mpc
/usr/lib/python3/dist-packages/setuptools/dist.py:723: UserWarning: Usage of dash-separated 'script-dir' will not be supported in future versions. Please use the underscore name 'script_dir' instead
  warnings.warn(
/usr/lib/python3/dist-packages/setuptools/dist.py:723: UserWarning: Usage of dash-separated 'install-scripts' will not be supported in future versions. Please use the underscore name 'install_scripts' instead
  warnings.warn(
---
Finished <<< mpc [1.73s]
--- stderr: lqr_pid                                                         
/usr/bin/ld: cannot find -ltf2: No such file or directory
/usr/bin/ld: cannot find -ltf2_ros: No such file or directory
/usr/bin/ld: cannot find -ltf2_geometry_msgs: No such file or directory
collect2: error: ld returned 1 exit status
gmake[2]: *** [CMakeFiles/lqr_pid.dir/build.make:230: lqr_pid] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:139: CMakeFiles/lqr_pid.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< lqr_pid [2.13s, exited with code 2]
Aborted  <<< realsense2_camera [0.52s]
Aborted  <<< joy_teleop [0.46s]
Aborted  <<< bringup [2.16s]
Aborted  <<< f1tenth_gym_ros [2.15s]
Aborted  <<< mouse_teleop [2.14s]
Aborted  <<< particle_filter [2.13s]
Aborted  <<< io_context [0.47s]
Aborted  <<< key_teleop [2.17s]
Aborted  <<< realsense2_description [0.51s]
Aborted  <<< urg_node2 [0.69s]
Aborted  <<< safety_node [2.14s]                                               
Aborted  <<< vesc_ackermann [0.47s]
Aborted  <<< slam_toolbox [2.17s]

Summary: 8 packages finished [2.55s]
  1 package failed: lqr_pid
  13 packages aborted: bringup f1tenth_gym_ros io_context joy_teleop key_teleop mouse_teleop particle_filter realsense2_camera realsense2_description safety_node slam_toolbox urg_node2 vesc_ackermann
  3 packages had stderr output: lqr_pid mpc safety_node
  6 packages not processed
