void LQRPID::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform(global_refFrame, msg->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        return;
    }

    // 위치 변환
    const auto& t = tf.transform.translation;
    const auto& r = tf.transform.rotation;
    tf2::Vector3 translation(t.x, t.y, t.z);
    tf2::Quaternion rotation(r.x, r.y, r.z, r.w);
    tf2::Transform tf2_transform(rotation, translation);

    tf2::Vector3 pos_orig(msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          msg->pose.pose.position.z);
    tf2::Vector3 pos_transformed = tf2_transform * pos_orig;

    double x = pos_transformed.x();
    double y = pos_transformed.y();

    // orientation 그대로 사용 (base_link 기준)
    const auto& q = msg->pose.pose.orientation;
    tf2::Quaternion q_orig(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);

    double velocity = msg->twist.twist.linear.x;

    // ✅ 차량이 반대 방향을 향하고 있다면 yaw 보정
    if (velocity < 0.0) {
        yaw += M_PI;
        yaw = pi2pi(yaw);
    }

    publish_message(x, y, yaw, velocity);
}






SteeringState LQRPID::lqr_steering(double x, double y, double yaw, double v) {
    auto [ind, e] = find_nearest_point(
        waypoints.x_m, waypoints.y_m, waypoints.psi_rad, x, y);

    // ✅ 속도 너무 작으면 제어하지 않음
    if (std::abs(v) < 0.1) {
        SteeringState s;
        s.steering_angle = 0.0;
        s.index = ind;
        s.lateral_error = e;
        s.yaw_error = 0.0;
        return s;
    }

    double k = waypoints.kappa_radpm[ind];
    double th_e = pi2pi(yaw - waypoints.psi_rad[ind]);

    // 로그로 확인
    RCLCPP_INFO(this->get_logger(), "LQR idx=%d, e=%.3f, th_e=%.3f, psi=%.3f, yaw=%.3f",
                ind, e, th_e, waypoints.psi_rad[ind], yaw);

    // 진행 방향 보정 (v > 0 정방향, v < 0 역방향)
    double direction = (v >= 0) ? 1.0 : -1.0;

    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
    A(0,0) = 1.0; A(0,1) = dt;
    A(1,2) = std::abs(v);  // 항상 양수
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

    SteeringState state;
    state.steering_angle = delta;
    state.index = ind;
    state.lateral_error = e;
    state.yaw_error = th_e;
    return state;
}
