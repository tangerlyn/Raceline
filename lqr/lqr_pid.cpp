// 전체 코드: global_refFrame을 'map'으로 복구 + TF transform 유지 + static TF 사용 전제
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
    // 글로벌 좌표계 map 기준 사용
    this->declare_parameter("global_refFrame", "map");
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
        tf_buffer_->transform(pose_in, pose_out, global_refFrame);  // 변환 대상: map
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

// 클래스 멤버에 포함되어야 할 것:
// private:
//   std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
//   std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

// 참고: static TF 브로드캐스트 명령어는 다음과 같음:
// ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map sim
// 이 명령어를 launch 파일 또는 터미널에 추가해서 좌표계 연결
