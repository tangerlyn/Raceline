void LQRPID::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    geometry_msgs::msg::TransformStamped tf;
    try {
        // odom frame에서 map frame으로의 변환을 가져옴
        tf = tf_buffer_->lookupTransform(global_refFrame, msg->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                    msg->header.frame_id.c_str(), global_refFrame.c_str(), ex.what());
        return;
    }

    // position 변환
    tf2::Vector3 original_pos(msg->pose.pose.position.x,
                              msg->pose.pose.position.y,
                              msg->pose.pose.position.z);
    tf2::Transform tf2_transform;
    tf2::fromMsg(tf.transform, tf2_transform);
    tf2::Vector3 transformed_pos = tf2_transform * original_pos;
    double x = transformed_pos.x();
    double y = transformed_pos.y();

    // orientation 변환
    tf2::Quaternion q_orig, q_rotated;
    tf2::fromMsg(msg->pose.pose.orientation, q_orig);
    q_rotated = tf2_transform.getRotation() * q_orig;
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_rotated).getRPY(roll, pitch, yaw);

    double velocity = msg->twist.twist.linear.x;

    // publish drive message
    publish_message(x, y, yaw, velocity);
}
