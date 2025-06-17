    double pi2pi(double angle);
    std::pair<int, double> find_nearest_point(
        const std::vector<double>& cx,
        const std::vector<double>& cy,
        const std::vector<double>& cyaw,
        double x, double y
    );


RCLCPP_ERROR(this->get_logger(), "Cannot Open CSV File: %s", waypoints_path.c_str());
