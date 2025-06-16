void LQRPID::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    geometry_msgs::msg::PoseStamped pose_in, pose_out;

    // ① 현재 odometry를 기반으로 PoseStamped 생성
    pose_in.header = msg->header;
    pose_in.pose = msg->pose.pose;

    // ② tf2를 이용해 global_refFrame(map) 기준으로 변환
    try {
        tf_buffer_->transform(pose_in, pose_out, global_refFrame);  // "map"
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        return;
    }

    // ③ 변환된 위치 좌표
    double x = pose_out.pose.position.x;
    double y = pose_out.pose.position.y;

    // ④ 방향 추출 (tf2::Quaternion 사용)
    tf2::Quaternion q_tf;
    q_tf.setX(pose_out.pose.orientation.x);
    q_tf.setY(pose_out.pose.orientation.y);
    q_tf.setZ(pose_out.pose.orientation.z);
    q_tf.setW(pose_out.pose.orientation.w);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);  // ⚠️ tf2의 Quaternion을 사용해야 함

    // ⑤ 속도 정보
    double velocity = msg->twist.twist.linear.x;

    // ⑥ 메시지 전송
    publish_message(x, y, yaw, velocity);
}
