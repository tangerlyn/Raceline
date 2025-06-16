// 전체 코드에서 TF 좌표계를 'sim' 기준으로 변환 적용
#include "lqr_pid/lqr_pid.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

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
    // 기존 파라미터 생략...
    this->declare_parameter("global_refFrame", "sim");  // map → sim 으로 변경
    ...
    global_refFrame = this->get_parameter("global_refFrame").as_string();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    load_waypoints();
}

void LQRPID::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
    geometry_msgs::msg::PoseStamped pose_in, pose_out;
    pose_in.header = msg->header;
    pose_in.pose = msg->pose.pose;

    try {
        tf_buffer_->transform(pose_in, pose_out, global_refFrame);
    } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        return;
    }

    double x = pose_out.pose.position.x;
    double y = pose_out.pose.position.y;

    tf2::Quaternion q(
        pose_out.pose.orientation.x,
        pose_out.pose.orientation.y,
        pose_out.pose.orientation.z,
        pose_out.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    double velocity = msg->twist.twist.linear.x;
    publish_message(x, y, yaw, velocity);
}

// class 내부에 다음 멤버도 추가되어 있어야 함:
// private:
//   std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
//   std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

// 나머지 publish_message, lqr_steering 등은 기존 내용 유지
