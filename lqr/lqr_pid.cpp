RCLCPP_INFO(this->get_logger(), "LQR idx=%d, e=%.3f, th_e=%.3f, psi=%.3f, yaw=%.3f",
            ind, e, th_e, waypoints.psi_rad[ind], yaw);


yaw += M_PI_2;
yaw = pi2pi(yaw);
