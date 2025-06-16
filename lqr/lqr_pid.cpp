// 주요 수정:
// 1. LQR: 속도 너무 작을 때 계산 skip
// 2. Waypoint 탐색: prev_idx 기반 n_ind_search 적용
// 3. PID gain scale 제거 or 제한

#include "lqr_pid/lqr_pid.hpp"
#include <math.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Eigen>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <limits>
#include <Eigen/Dense>
#include <functional>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

LQRPID::LQRPID() : Node("lqr_pid_node"), prev_idx(0) {
    ... // (기존 파라미터 declare 및 get_parameter 생략)
    ...
    load_waypoints();
}

std::pair<int, double> LQRPID::find_nearest_point(const std::vector<double>& cx,
                                                  const std::vector<double>& cy,
                                                  const std::vector<double>& cyaw,
                                                  double x, double y) {
    int ind = prev_idx;
    double min_d2 = std::numeric_limits<double>::infinity();
    int start = std::max(0, prev_idx);
    int end = std::min((int)cx.size(), prev_idx + n_ind_search);

    for (int i = start; i < end; ++i) {
        double dx = cx[i] - x;
        double dy = cy[i] - y;
        double d2 = dx * dx + dy * dy;
        if (d2 < min_d2) {
            min_d2 = d2;
            ind = i;
        }
    }

    double e = std::sqrt(min_d2);
    double angle = pi2pi(cyaw[ind] - std::atan2(cy[ind] - y, cx[ind] - x));
    if (angle < 0.0) e = -e;

    prev_idx = ind;
    return {ind, e};
}

double LQRPID::pid_velocity(double x, double y, double yaw, double v) {
    (void)yaw;
    constexpr double epsilon = 1e-4;

    auto [idx, _] = find_nearest_point(waypoints.x_m, waypoints.y_m, waypoints.psi_rad, x, y);
    double v_ref = waypoints.vx_mps[idx];
    double error = v_ref - v;

    velocity_error_sum += error * dt;
    double d_error = (error - prev_velocity_error) / dt;

    double output = pid_kp * error + pid_ki * velocity_error_sum + pid_kd * d_error;
    output = std::clamp(output, 0.0, max_speed);
    prev_velocity_error = error;

    return output;
}

SteeringState LQRPID::lqr_steering(double x, double y, double yaw, double v) {
    if (v < 1e-3) {
        RCLCPP_WARN(this->get_logger(), "Velocity too small for LQR. Using zero steering.");
        return SteeringState{0.0, prev_idx, 0.0, 0.0};
    }

    auto [ind, e] = find_nearest_point(waypoints.x_m, waypoints.y_m, waypoints.psi_rad, x, y);
    double k = waypoints.kappa_radpm[ind];
    double th_e = pi2pi(yaw - waypoints.psi_rad[ind]);

    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
    A(0,0) = 1.0; A(0,1) = dt;
    A(1,2) = v;
    A(2,2) = 1.0; A(2,3) = dt;
    A(3,3) = 1.0;

    Eigen::Matrix<double,4,1> B = Eigen::Matrix<double,4,1>::Zero();
    B(3,0) = v / wheelbase;

    Eigen::Matrix4d Q = Eigen::Matrix4d::Zero();
    Q(0,0) = lqr_q_y;
    Q(1,1) = 1.0;
    Q(2,2) = lqr_q_yaw;
    Q(3,3) = 1.0;

    Eigen::Matrix<double,1,1> R;
    R(0,0) = lqr_r_steer;

    Eigen::Matrix<double,1,4> K = dlqr(A, B, Q, R);

    Eigen::Vector4d x_err;
    x_err << e,
             (e - prev_lateral_error_) / dt,
             th_e,
             (th_e - prev_yaw_error_) / dt;

    double ff = std::atan2(wheelbase * k, 1.0);
    double fb = pi2pi(- (K * x_err)(0));
    double delta = std::clamp(ff + fb, -max_steer_rad, max_steer_rad);

    prev_lateral_error_ = e;
    prev_yaw_error_ = th_e;

    return SteeringState{delta, ind, e, th_e};
}

// 나머지 publish_message, odom_callback, timer_callback, main()는 그대로 유지하되
// prev_idx는 class의 멤버 변수로 선언 필요:
// private:
//     int prev_idx;
