void LQRPID::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform(global_refFrame, msg->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        return;
    }

    // 위치 변환만 적용
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

    // yaw는 odom 그대로 사용
    const auto& q = msg->pose.pose.orientation;
    tf2::Quaternion q_orig(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_orig).getRPY(roll, pitch, yaw);

    double velocity = msg->twist.twist.linear.x;

    publish_message(x, y, yaw, velocity);
}
