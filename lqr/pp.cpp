nstead
  warnings.warn(
Finished <<< teleop_tools [0.42s]                                                                  
--- stderr: f1tenth_stack                                                                 
/usr/lib/python3/dist-packages/setuptools/dist.py:723: UserWarning: Usage of dash-separated 'script-dir' will not be supported in future versions. Please use the underscore name 'script_dir' instead
  warnings.warn(
---
Finished <<< f1tenth_stack [1.31s]
--- stderr: pure_pursuit                                 
In file included from /opt/ros/humble/include/rclcpp/rclcpp/logging.hpp:24,
                 from /opt/ros/humble/include/rclcpp/rclcpp/client.hpp:40,
                 from /opt/ros/humble/include/rclcpp/rclcpp/callback_group.hpp:24,
                 from /opt/ros/humble/include/rclcpp/rclcpp/any_executable.hpp:20,
                 from /opt/ros/humble/include/rclcpp/rclcpp/memory_strategy.hpp:25,
                 from /opt/ros/humble/include/rclcpp/rclcpp/memory_strategies.hpp:18,
                 from /opt/ros/humble/include/rclcpp/rclcpp/executor_options.hpp:20,
                 from /opt/ros/humble/include/rclcpp/rclcpp/executor.hpp:37,
                 from /opt/ros/humble/include/rclcpp/rclcpp/executors/multi_threaded_executor.hpp:25,
                 from /opt/ros/humble/include/rclcpp/rclcpp/executors.hpp:21,
                 from /opt/ros/humble/include/rclcpp/rclcpp/rclcpp.hpp:155,
                 from /opt/ros/humble/include/tf2_ros/tf2_ros/buffer_interface.h:48,
                 from /opt/ros/humble/include/tf2_ros/tf2_ros/buffer.h:42,
                 from /home/misys/f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/include/pure_pursuit.hpp:6,
                 from /home/misys/f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/src/pure_pursuit.cpp:1:
/home/misys/f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/src/pure_pursuit.cpp: In member function ‘void PurePursuit::load_waypoints()’:
/home/misys/f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/src/pure_pursuit.cpp:93:42: warning: format ‘%s’ expects argument of type ‘char*’, but argument 5 has type ‘std::string’ {aka ‘std::__cxx11::basic_string<char>’} [-Wformat=]
   93 |         RCLCPP_ERROR(this->get_logger(), "Cannot Open CSV File: %s", waypoints_path);
      |                                          ^~~~~~~~~~~~~~~~~~~~~~~~~~
/home/misys/f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/src/pure_pursuit.cpp:93:66: note: format string is defined here
   93 |         RCLCPP_ERROR(this->get_logger(), "Cannot Open CSV File: %s", waypoints_path);
      |                                                                 ~^
      |                                                                  |
      |                                                                  char*
In file included from /opt/ros/humble/include/rclcpp/rclcpp/logging.hpp:24,
                 from /opt/ros/humble/include/rclcpp/rclcpp/client.hpp:40,
                 from /opt/ros/humble/include/rclcpp/rclcpp/callback_group.hpp:24,
                 from /opt/ros/humble/include/rclcpp/rclcpp/any_executable.hpp:20,
                 from /opt/ros/humble/include/rclcpp/rclcpp/memory_strategy.hpp:25,
                 from /opt/ros/humble/include/rclcpp/rclcpp/memory_strategies.hpp:18,
                 from /opt/ros/humble/include/rclcpp/rclcpp/executor_options.hpp:20,
                 from /opt/ros/humble/include/rclcpp/rclcpp/executor.hpp:37,
                 from /opt/ros/humble/include/rclcpp/rclcpp/executors/multi_threaded_executor.hpp:25,
                 from /opt/ros/humble/include/rclcpp/rclcpp/executors.hpp:21,
                 from /opt/ros/humble/include/rclcpp/rclcpp/rclcpp.hpp:155,
                 from /opt/ros/humble/include/tf2_ros/tf2_ros/buffer_interface.h:48,
                 from /opt/ros/humble/include/tf2_ros/tf2_ros/buffer.h:42,
                 from /home/misys/f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/include/pure_pursuit.hpp:6,
                 from /home/misys/f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/src/pure_pursuit.cpp:1:
/home/misys/f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/src/pure_pursuit.cpp:123:37: warning: format ‘%s’ expects argument of type ‘char*’, but argument 6 has type ‘std::string’ {aka ‘std::__cxx11::basic_string<char>’} [-Wformat=]
  123 |     RCLCPP_INFO(this->get_logger(), "Finished loading %d waypoints from %s", num_waypoints, waypoints_path);
      |                                     ^~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/home/misys/f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/src/pure_pursuit.cpp:123:74: note: format string is defined here
  123 |     RCLCPP_INFO(this->get_logger(), "Finished loading %d waypoints from %s", num_waypoints, waypoints_path);
      |                                                                         ~^
      |                                                                          |
      |                                                                          char*
/home/misys/f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/src/pure_pursuit.cpp: At global scope:
/home/misys/f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/src/pure_pursuit.cpp:367:8: error: no declaration matches ‘double PurePursuit::pi2pi(double)’
  367 | double PurePursuit::pi2pi(double angle) {
      |        ^~~~~~~~~~~
/home/misys/f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/src/pure_pursuit.cpp:367:8: note: no functions named ‘double PurePursuit::pi2pi(double)’
In file included from /home/misys/f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/src/pure_pursuit.cpp:1:
/home/misys/f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/include/pure_pursuit.hpp:32:7: note: ‘class PurePursuit’ defined here
   32 | class PurePursuit : public rclcpp::Node {
      |       ^~~~~~~~~~~
/home/misys/f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/src/pure_pursuit.cpp:374:24: error: no declaration matches ‘std::pair<int, double> PurePursuit::find_nearest_point(const std::vector<double, std::allocator<double> >&, const std::vector<double, std::allocator<double> >&, const std::vector<double, std::allocator<double> >&, double, double)’
  374 | std::pair<int, double> PurePursuit::find_nearest_point(
      |                        ^~~~~~~~~~~
/home/misys/f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/src/pure_pursuit.cpp:374:24: note: no functions named ‘std::pair<int, double> PurePursuit::find_nearest_point(const std::vector<double, std::allocator<double> >&, const std::vector<double, std::allocator<double> >&, const std::vector<double, std::allocator<double> >&, double, double)’
In file included from /home/misys/f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/src/pure_pursuit.cpp:1:
/home/misys/f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/include/pure_pursuit.hpp:32:7: note: ‘class PurePursuit’ defined here
   32 | class PurePursuit : public rclcpp::Node {
      |       ^~~~~~~~~~~
/home/misys/f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/src/pure_pursuit.cpp: In member function ‘void PurePursuit::odom_callback(nav_msgs::msg::Odometry_<std::allocator<void> >::ConstSharedPtr)’:
/home/misys/f1tenth_ws/src/CL2-UWaterloo/pure_pursuit/src/pure_pursuit.cpp:409:31: error: ‘find_nearest_point’ was not declared in this scope
  409 |     auto [nearest_idx, cte] = find_nearest_point(waypoints.X, waypoints.Y, dummy_yaw, x_car_world, y_car_world);
      |                               ^~~~~~~~~~~~~~~~~~
gmake[2]: *** [CMakeFiles/pure_pursuit.dir/build.make:76: CMakeFiles/pure_pursuit.dir/src/pure_pursuit.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:139: CMakeFiles/pure_pursuit.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< pure_pursuit [10.3s, exited with code 2]

Summary: 27 packages finished [10.9s]
  1 package failed: pure_pursuit
  6 packages had stderr output: f1tenth_gym_ros f1tenth_stack mpc particle_filter pure_pursuit safety_node
