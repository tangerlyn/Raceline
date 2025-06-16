SteeringState LQRPID::lqr_steering(double x, double y, double yaw, double v) {
    // 1. 최근접 인덱스 + preview 적용
    auto [ind, e] = find_nearest_point(
        waypoints.x_m, waypoints.y_m, waypoints.psi_rad, x, y);

    // 2. 추종 대상 waypoint의 방향 및 곡률
    double psi = waypoints.psi_rad[ind];
    double k = waypoints.kappa_radpm[ind];
    double th_e = pi2pi(yaw - psi);  // heading error

    // 3. 속도 조건 검사
    if (std::abs(v) < 0.1) {
        SteeringState s;
        s.steering_angle = 0.0;
        s.index = ind;
        s.lateral_error = e;
        s.yaw_error = th_e;
        return s;
    }

    // 4. 상태-입력 행렬 정의
    double direction = (v >= 0) ? 1.0 : -1.0;

    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
    A(0,0) = 1.0; A(0,1) = dt;
    A(1,2) = std::abs(v);  // always positive
    A(2,2) = 1.0; A(2,3) = dt;
    A(3,3) = 1.0;

    Eigen::Matrix<double,4,1> B = Eigen::Matrix<double,4,1>::Zero();
    B(3,0) = direction * std::abs(v) / wheelbase;

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

    x_err *= direction;

    double ff = std::atan2(wheelbase * k, 1.0);
    double fb = pi2pi(- (K * x_err)(0));
    double delta = std::clamp(ff + fb, -max_steer_rad, max_steer_rad);

    prev_lateral_error_ = e;
    prev_yaw_error_ = th_e;

    // 디버깅 출력
    RCLCPP_INFO(this->get_logger(), "LQR idx=%d, e=%.3f, th_e=%.3f, psi=%.3f, yaw=%.3f, delta=%.3f",
                ind, e, th_e, psi, yaw, delta);

    SteeringState state;
    state.steering_angle = delta;
    state.index = ind;
    state.lateral_error = e;
    state.yaw_error = th_e;
    return state;
}
