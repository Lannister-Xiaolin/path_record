//
// Created by zxl on 2022/4/26.
//

#include "path_record_server.h"
#include <chrono>

using namespace std::chrono_literals; // NOLINT
using namespace std::chrono;  // NOLINT

PathRecordServer::PathRecordServer() : Node("path_record_server") {
  const std::string service_prefix = get_name() + std::string("/");
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("recorded_path", 1);
  path_record_server_ = this->create_service<path_record::srv::PathRecord>(service_prefix + "path_record", std::bind(
      &PathRecordServer::HandleService, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  this->declare_parameter("tracking_frame", "base_footprint");
  this->declare_parameter("map_frame", "map");
  this->declare_parameter("dist_thresh", 0.20);
  this->declare_parameter("angle_thresh", 1.221);
  // todo 是否根据时间间隔来累加,目前暂无必要性？
  this->declare_parameter("time_thresh", -1.);
  this->declare_parameter("frequency", 1.);
  this->declare_parameter("path_pub_frequency", 1.);
}

void PathRecordServer::HandleService(const std::shared_ptr<rmw_request_id_t> request_header,
                                     const std::shared_ptr<path_record::srv::PathRecord::Request> request,
                                     const std::shared_ptr<path_record::srv::PathRecord::Response> response) {
  if (record_timer_ == nullptr) {
    UpdateParam();
    record_timer_ =
        this->create_wall_timer(std::chrono::duration<double>{record_interval_},
                                std::bind(&PathRecordServer::RecordPath, this));
    last_warning_time_ = this->now();
    last_pub_time_ = this->now();
  }

  if (request->request_code == request->STOP && working_status_) {
    RCLCPP_INFO(get_logger(), "--- Stop path record loop!!!");
    working_status_ = false;
  } else if (request->request_code == request->RESET) {
    RCLCPP_INFO(get_logger(), "--- Reset path record loop!!!");
    record_path_ = nav_msgs::msg::Path();
    UpdateParam();
    working_status_ = true;
  } else if (request->request_code == request->START && !working_status_) {
    RCLCPP_INFO(get_logger(), "✓✓✓ Start path record loop!!!");
    UpdateParam();
    working_status_ = true;
  } else if (request->request_code == request->GET) {
    RCLCPP_INFO(get_logger(), "✓✓✓ Get recorded path, path size: %d!!!", record_path_.poses.size());
    response->record_path = record_path_;
    response->last_pose_index = record_path_.poses.size() - 1;
  }

}
void PathRecordServer::RecordPath() {
  if (!working_status_) return;
  auto now = this->now();
  try {
    //hint: foxy with timeout param with block if no valid tf data in buffer!!!
    auto transform_stamped = tf_buffer_->lookupTransform(
        map_frame_, tracking_frame_, now, tf2::durationFromSec(0.1));
    current_pose_ = ToPoseStamped(transform_stamped);
    if (record_path_.poses.empty()) { record_path_.poses.push_back(current_pose_); }
    auto relative_x = current_pose_.pose.position.x - record_path_.poses.back().pose.position.x;
    auto relative_y = current_pose_.pose.position.y - record_path_.poses.back().pose.position.y;
    if (std::hypot(relative_x, relative_y) > dist_thresh_) {
      record_path_.poses.push_back(current_pose_);
    } else if (angle_thresh_ > 0) {
      auto &quat1 = current_pose_.pose.orientation;
      auto &quat2 = record_path_.poses.back().pose.orientation;
      if (tf2::angleShortestPath(tf2::Quaternion(quat1.x, quat1.y, quat1.z, quat1.w),
                                 tf2::Quaternion(quat2.x, quat2.y, quat2.z, quat2.w)) > angle_thresh_)
        record_path_.poses.push_back(current_pose_);
    }
    if ((now - last_pub_time_).seconds() > path_pub_interval_) {
      path_pub_->publish(record_path_);
      last_pub_time_ = now;
    }

  } catch (tf2::TransformException &ex) {
    if ((now - last_warning_time_).seconds() > 1.0) {
      RCLCPP_WARN(
          this->get_logger(), "Could not transform %s to %s: %s",
          map_frame_.c_str(), tracking_frame_.c_str(), ex.what());
      last_warning_time_ = now;
    }
    return;
  }
}
void PathRecordServer::UpdateParam() {
  map_frame_ = get_parameter("map_frame").as_string();
  tracking_frame_ = get_parameter("tracking_frame").as_string();
  dist_thresh_ = get_parameter("dist_thresh").as_double();
  angle_thresh_ = get_parameter("angle_thresh").as_double();
  time_thresh_ = get_parameter("time_thresh").as_double();
  record_interval_ = 1.0 / get_parameter("frequency").as_double();
  path_pub_interval_ = 1.0 / get_parameter("path_pub_frequency").as_double();
  record_path_.header.frame_id = map_frame_;
  RCLCPP_INFO(get_logger(), "--- Update param success!!!");
}
geometry_msgs::msg::PoseStamped ToPoseStamped(const geometry_msgs::msg::TransformStamped &transform_stamped) {
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = transform_stamped.header.frame_id;
  pose_stamped.header.stamp = transform_stamped.header.stamp;
  pose_stamped.pose.position.x = transform_stamped.transform.translation.x;
  pose_stamped.pose.position.y = transform_stamped.transform.translation.y;
  pose_stamped.pose.orientation.x = transform_stamped.transform.rotation.x;
  pose_stamped.pose.orientation.y = transform_stamped.transform.rotation.y;
  pose_stamped.pose.orientation.z = transform_stamped.transform.rotation.z;
  pose_stamped.pose.orientation.w = transform_stamped.transform.rotation.w;
  return pose_stamped;
}
