void LQRPID::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    // Try to get transform from odom frame (e.g. "ego_racecar/odom") to global frame (e.g. "map")
    geometry_msgs::msg::TransformStamped tf;
    try {
        tf = tf_buffer_->lookupTransform(global_refFrame, msg->header.frame_id, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed from %s to %s: %s",
                    msg->header.frame_id.c_str(), global_refFrame.c_str(), ex.what());
        return;
    }

    // 1. Create tf2::Transform manually from geometry_msgs::Transform
    const auto& t = tf.transform.translation;
    const auto& r = tf.transform.rotation;
    tf2::Vector3 translation(t.x, t.y, t.z);
    tf2::Quaternion rotation(r.x, r.y, r.z, r.w);
    tf2::Transform tf2_transform(rotation, translation);

    // 2. Transform position
    tf2::Vector3 pos_orig(msg->pose.pose.position.x,
                          msg->pose.pose.position.y,
                          msg->pose.pose.position.z);
    tf2::Vector3 pos_transformed = tf2_transform * pos_orig;

    double x = pos_transformed.x();
    double y = pos_transformed.y();

    // 3. Transform orientation
    const auto& q = msg->pose.pose.orientation;
    tf2::Quaternion q_orig(q.x, q.y, q.z, q.w);
    tf2::Quaternion q_rotated = rotation * q_orig;  // Combine rotations

    double roll, pitch, yaw;
    tf2::Matrix3x3(q_rotated).getRPY(roll, pitch, yaw);

    // 4. Extract velocity (assumed to be in the original base_link frame)
    double velocity = msg->twist.twist.linear.x;

    // 5. Pass transformed pose and velocity to LQR logic
    publish_message(x, y, yaw, velocity);
}
