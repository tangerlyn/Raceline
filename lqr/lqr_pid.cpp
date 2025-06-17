bool has_started_ = false;
bool has_finished_ = false;
rclcpp::Time start_time_;
rclcpp::Time end_time_;

double prev_x_ = std::numeric_limits<double>::max();
double prev_y_ = std::numeric_limits<double>::max();

// 결승선 정의
const double finish_line_x_ = 1.0;
const double finish_line_y_min_ = -1.0;
const double finish_line_y_max_ = 1.0;




// 결승선 y 범위 안에 있는가?
bool in_finish_line_zone = 
    (y > finish_line_y_min_) && (y < finish_line_y_max_);

// x가 결승선 x를 넘었는지 검사 (왼쪽 → 오른쪽 통과)
bool crossed_finish_line = 
    in_finish_line_zone &&
    (prev_x_ < finish_line_x_ && x >= finish_line_x_);

if (crossed_finish_line) {
    if (!has_started_) {
        has_started_ = true;
        start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "[Lap] Lap timing started");
    } else if (!has_finished_) {
        rclcpp::Duration elapsed = this->now() - start_time_;
        if (elapsed.seconds() > 5.0) {  // 처음 진입 이후 5초는 지나야 유효한 랩
            has_finished_ = true;
            end_time_ = this->now();
            double lap_time = elapsed.seconds();
            RCLCPP_INFO(this->get_logger(), "[Lap] Lap timing finished: %.3f seconds", lap_time);
        }
    }
}

// 이전 위치 저장
prev_x_ = x;
prev_y_ = y;
