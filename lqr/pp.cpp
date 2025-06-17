bool has_started_ = false;
bool has_finished_ = false;
rclcpp::Time start_time_;
rclcpp::Time end_time_;

double start_x_ = 0.0;
double start_y_ = 0.0;




double dx = x_car_world - start_x_;
double dy = y_car_world - start_y_;
double dist_to_start = std::sqrt(dx * dx + dy * dy);

// 랩 시작
if (!has_started_) {
    start_time_ = this->now();
    has_started_ = true;
    RCLCPP_INFO(this->get_logger(), "Lap started!");
}

// 랩 종료 (1초 이상 지나고, 출발점에 근접했을 때)
if (has_started_ && !has_finished_ &&
    dist_to_start < 1.0 && (this->now() - start_time_).seconds() > 1.0) {
    end_time_ = this->now();
    has_finished_ = true;

    double lap_time = (end_time_ - start_time_).seconds();
    RCLCPP_INFO(this->get_logger(),
        "\n========== LAP COMPLETE ==========\n"
        "Lap time: %.2f seconds\n"
        "==================================", lap_time);
}
