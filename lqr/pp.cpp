// ===== 수정 사항 요약 =====
// 1. find_nearest_point(), pi2pi() 함수 추가
// 2. odom_callback() 확장: 랩타임 측정 + 평균 CTE 계산
// 3. 관련 멤버 변수들 PurePursuit 클래스에 선언 필요

#include <cmath>
#include <limits>

// === PurePursuit 클래스 내부 ===

// 랩타임 및 CTE 통계 변수 (pure_pursuit.hpp에 멤버로 추가해야 함)
bool has_started_ = false;
bool has_finished_ = false;
rclcpp::Time start_time_;
rclcpp::Time end_time_;
double start_x_ = 1.0;
double start_y_ = 0.0;
double total_cte_ = 0.0;
int cte_count_ = 0;

// === pi2pi 함수 ===
double PurePursuit::pi2pi(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// === find_nearest_point 함수 ===
std::pair<int, double> PurePursuit::find_nearest_point(
    const std::vector<double>& cx,
    const std::vector<double>& cy,
    const std::vector<double>& cyaw,
    double x, double y) {

    int nearest_ind = 0;
    double min_d2 = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < cx.size(); ++i) {
        double dx = cx[i] - x;
        double dy = cy[i] - y;
        double d2 = dx * dx + dy * dy;
        if (d2 < min_d2) {
            min_d2 = d2;
            nearest_ind = i;
        }
    }

    double e = std::sqrt(min_d2);
    double angle = std::atan2(cy[nearest_ind] - y, cx[nearest_ind] - x);
    double yaw_diff = pi2pi(cyaw[nearest_ind] - angle);

    if (yaw_diff < 0.0) e = -e;
    return {nearest_ind, e};
}

// === odom_callback 확장 ===
void PurePursuit::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj) {
    x_car_world = odom_submsgObj->pose.pose.position.x;
    y_car_world = odom_submsgObj->pose.pose.position.y;

    // CTE 계산
    std::vector<double> dummy_yaw(waypoints.X.size(), 0.0);  // yaw 정보 없으면 0으로 채움
    auto [nearest_idx, cte] = find_nearest_point(waypoints.X, waypoints.Y, dummy_yaw, x_car_world, y_car_world);

    total_cte_ += std::abs(cte);
    cte_count_++;

    // 랩타임 측정
    double dx = x_car_world - start_x_;
    double dy = y_car_world - start_y_;
    double dist_to_start = std::sqrt(dx * dx + dy * dy);

    if (!has_started_) {
        start_time_ = this->now();
        has_started_ = true;
        RCLCPP_INFO(this->get_logger(), "Lap started.");
    }

    if (has_started_ && !has_finished_ && dist_to_start < 1.0 && (this->now() - start_time_).seconds() > 1.0) {
        end_time_ = this->now();
        has_finished_ = true;

        double lap_time = (end_time_ - start_time_).seconds();
        double avg_cte = (cte_count_ > 0) ? total_cte_ / cte_count_ : 0.0;

        RCLCPP_INFO(this->get_logger(),
            "\n========== LAP COMPLETE ==========\n"
            "Lap time: %.2f seconds\n"
            "Average CTE: %.3f meters (from %d samples)\n"
            "==================================",
            lap_time, avg_cte, cte_count_);
    }

    // 원래 로직 수행
    get_waypoint();
    transformandinterp_waypoint();
    double steering_angle = p_controller();
    publish_message(steering_angle);
}
