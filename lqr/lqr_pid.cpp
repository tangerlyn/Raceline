void LQRPID::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    // 현재 위치를 pose_in (map 기준)으로 정의
    Eigen::Vector3d position_world(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z);

    // TF 조회: map → base_link
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        transformStamped = tf_buffer_->lookupTransform(
            car_refFrame, global_refFrame, tf2::TimePointZero);  // "base_link", "map"
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
        return;
    }

    // 회전 행렬 (Quaternion → Eigen)
    Eigen::Quaterniond q(
        transformStamped.transform.rotation.w,
        transformStamped.transform.rotation.x,
        transformStamped.transform.rotation.y,
        transformStamped.transform.rotation.z);
    Eigen::Matrix3d rotation = q.toRotationMatrix();

    // translation 벡터
    Eigen::Vector3d translation(
        transformStamped.transform.translation.x,
        transformStamped.transform.translation.y,
        transformStamped.transform.translation.z);

    // 좌표 변환: pose_world → pose_car 기준
    Eigen::Vector3d position_car = rotation * position_world + translation;

    // yaw 추출
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // 속도 그대로 사용
    double velocity = msg->twist.twist.linear.x;

    // publish
    publish_message(position_car.x(), position_car.y(), yaw, velocity);
}
