//
// Created by zxl on 2022/4/26.
//

#ifndef COVERAGE_MAP_2D_SRC_PATH_RECORD_SERVER_MAIN_H_
#define COVERAGE_MAP_2D_SRC_PATH_RECORD_SERVER_MAIN_H_
#include <string>
#include <memory>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "path_record/srv/path_record.hpp"
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.h>

geometry_msgs::msg::PoseStamped ToPoseStamped(const geometry_msgs::msg::TransformStamped& transform_stamped);


/****
 * Reference from navigation2, using nav2_util::LifecycleNode for lifecycle manage
 */
class PathRecordServer : public rclcpp::Node {
 public:

  PathRecordServer();

 private:

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<path_record::srv::PathRecord>::SharedPtr path_record_server_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string tracking_frame_;
  std::string map_frame_;
  rclcpp::TimerBase::SharedPtr record_timer_{nullptr};
  void RecordPath();
  bool working_status_{false};
  nav_msgs::msg::Path record_path_;
  double dist_thresh_;
  double angle_thresh_;
  double time_thresh_;
  double record_interval_;
  double path_pub_interval_;
  geometry_msgs::msg::PoseStamped current_pose_;
  rclcpp::Time last_warning_time_,last_pub_time_;
  void HandleService(
      const std::shared_ptr<rmw_request_id_t> request_header,
      const std::shared_ptr<path_record::srv::PathRecord::Request> request,
      const std::shared_ptr<path_record::srv::PathRecord::Response> response);
  void UpdateParam();
// protected:
//  /**
//   * @brief Sets up required params and services. Loads map and its parameters from the file
//   * @param state Lifecycle Node's state
//   * @return Success or Failure
//   */
//  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
//  /**
//   * @brief Start publishing the map using the latched topic
//   * @param state Lifecycle Node's state
//   * @return Success or Failure
//   */
//  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
//  /**
//   * @brief Stops publishing the latched topic
//   * @param state Lifecycle Node's state
//   * @return Success or Failure
//   */
//  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
//  /**
//   * @brief Resets the member variables
//   * @param state Lifecycle Node's state
//   * @return Success or Failure
//   */
//  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
//  /**
//   * @brief Called when in Shutdown state
//   * @param state Lifecycle Node's state
//   * @return Success or Failure
//   */
//  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;
//
//  /**
//   * @brief Method correcting msg_ header when it belongs to instantiated object
//   */
//  void updateMsgHeader();
//
//  /**
//   * @brief Map getting service callback
//   * @param request_header Service request header
//   * @param request Service request
//   * @param response Service response
//   */
//  void getCoveragePathsByFileCallback(
//      const std::shared_ptr<rmw_request_id_t> request_header,
//      const std::shared_ptr<coverage_planner::srv::CoveragePathsByFile::Request> request,
//      std::shared_ptr<coverage_planner::srv::CoveragePathsByFile::Response> response);
//
//  /**
//   * @brief Map loading service callback
//   * @param request_header Service request header
//   * @param request Service request
//   * @param response Service response
//   */
//  void getCoveragePathsByGridCallback(
//      const std::shared_ptr<rmw_request_id_t> request_header,
//      const std::shared_ptr<coverage_planner::srv::CoveragePathsByGrid::Request> request,
//      std::shared_ptr<coverage_planner::srv::CoveragePathsByGrid::Response> response);
//
//  rclcpp::Service<coverage_planner::srv::CoveragePathsByGrid>::SharedPtr grid_service_;
//  rclcpp::Service<coverage_planner::srv::CoveragePathsByFile>::SharedPtr file_service_;
// private:
//  std::shared_ptr<CoverageRos2Adapter> coverage_planner_ptr_;

};

#endif //COVERAGE_MAP_2D_SRC_PATH_RECORD_SERVER_MAIN_H_
