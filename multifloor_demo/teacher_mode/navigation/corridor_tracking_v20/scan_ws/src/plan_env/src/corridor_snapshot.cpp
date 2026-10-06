// V20 shadow-only data export. No planner, optimizer or occupancy decisions are
// changed here. All callbacks run in the existing SingleThreadedExecutor.
#include "plan_env/grid_map.h"
#include <iomanip>
#include <sstream>

void GridMap::initCorridorSnapshot() {
  corridor_observed_stamp_ns_.assign(md_.occupancy_buffer_.size(), 0);
  corridor_snapshot_pub_ = node_->create_publisher<std_msgs::msg::String>(
      "/demo/teacher/corridor/snapshot", 1);
  corridor_request_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
      "/demo/teacher/corridor/request", 1,
      std::bind(&GridMap::corridorRequest, this, std::placeholders::_1));
  corridor_snapshot_timer_ = node_->create_wall_timer(std::chrono::milliseconds(500),
      std::bind(&GridMap::publishCorridorSnapshot, this));
}

void GridMap::corridorRequest(const nav_msgs::msg::Path::ConstSharedPtr &request) {
  corridor_request_points_.clear();
  corridor_request_stamp_ns_ = rclcpp::Time(request->header.stamp).nanoseconds();
  corridor_request_error_.clear();
  if (request->header.frame_id != mp_.frame_id_ || request->poses.empty() ||
      request->poses.size() > 128 || corridor_request_stamp_ns_ <= 0) {
    corridor_request_error_ = "invalid_frame_empty_or_oversized_request"; return;
  }
  for (const auto &pose : request->poses) {
    Eigen::Vector3d p(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    if (!p.allFinite() || p.cwiseAbs().maxCoeff() > 10000.) { corridor_request_error_ = "nonfinite_or_out_of_range_request"; corridor_request_points_.clear(); return; }
    corridor_request_points_.push_back(p);
  }
}

void GridMap::publishCorridorSnapshot() {
  if (!corridor_snapshot_pub_->get_subscription_count() || corridor_request_stamp_ns_ <= 0) return;
  const int64_t now_ns = node_->now().nanoseconds();
  std::ostringstream out; out << std::setprecision(17);
  // The configured frame is required to be the known SLAM frame by the runtime
  // binding. Avoid accepting arbitrary strings into the JSON producer.
  const bool frame_valid = mp_.frame_id_ == "camera_init";
  out << "{\"schema\":\"teacher_scan_local_snapshot/v1\",\"frame_id\":\"camera_init\","
      << "\"source\":\"actual_registered_lidar_raycast\",\"shadow_only\":true,"
      << "\"revision\":" << corridor_revision_ << ",\"stamp_ns\":" << now_ns
      << ",\"source_cloud_stamp_ns\":" << corridor_fused_cloud_stamp_ns_
      << ",\"source_sensor_pose_stamp_ns\":" << corridor_fused_pose_stamp_ns_
      << ",\"request_stamp_ns\":" << corridor_request_stamp_ns_
      << ",\"resolution_m\":" << mp_.resolution_;
  auto fail = [&](const char *reason) {
    out << ",\"complete\":false,\"reason\":\"" << reason << "\"}";
    std_msgs::msg::String msg; msg.data = out.str(); corridor_snapshot_pub_->publish(msg);
  };
  if (!frame_valid) { fail("unsupported_frame"); return; }
  if (!corridor_request_error_.empty() || corridor_request_points_.empty()) { fail("invalid_request"); return; }
  if (corridor_fused_cloud_stamp_ns_ <= 0 || corridor_surface_request_stamp_ns_ != corridor_request_stamp_ns_) {
    fail("awaiting_fusion_for_request"); return;
  }
  if (now_ns < corridor_request_stamp_ns_ || now_ns - corridor_request_stamp_ns_ > 3000000000LL) {
    fail("request_expired"); return;
  }
  Eigen::Vector3d lower = corridor_request_points_.front(), upper = lower;
  for (const auto &p : corridor_request_points_) { lower = lower.cwiseMin(p); upper = upper.cwiseMax(p); }
  if ((upper-lower).maxCoeff() > 5.) { fail("requested_extent_too_large"); return; }
  lower -= Eigen::Vector3d(.9, .9, .75); upper += Eigen::Vector3d(.9, .9, .75);
  Eigen::Vector3i lo, hi; posToIndex(lower, lo); posToIndex(upper, hi);
  const Eigen::Vector3i shape = hi - lo + Eigen::Vector3i::Ones();
  const int64_t count = static_cast<int64_t>(shape.x()) * shape.y() * shape.z();
  if ((shape.array() <= 0).any() || count > 65536) { fail("requested_voxel_budget_exceeded"); return; }
  out << ",\"origin_index_xyz\":[" << lo.x() << ',' << lo.y() << ',' << lo.z()
      << "],\"shape_xyz\":[" << shape.x() << ',' << shape.y() << ',' << shape.z()
      << "],\"state_encoding\":\"x_y_z_dense_0_unknown_1_strong_observed_free_2_occupied\",\"states\":\"";
  std::vector<int64_t> ages; ages.reserve(static_cast<size_t>(count));
  for (int x=lo.x(); x<=hi.x(); ++x) for (int y=lo.y(); y<=hi.y(); ++y) for (int z=lo.z(); z<=hi.z(); ++z) {
    Eigen::Vector3i id(x,y,z); char state='0'; int64_t age=-1;
    if (isInMap(id)) {
      const int address=toAddress(id); const double odds=md_.occupancy_buffer_[address];
      const auto observed=corridor_observed_stamp_ns_[address];
      if (observed > 0 && observed <= now_ns) age=(now_ns-observed+999999LL)/1000000LL;
      if (odds > mp_.min_occupancy_log_) state='2';
      else if (age >= 0 && odds >= mp_.clamp_min_log_-1e-9 && odds <= mp_.clamp_min_log_+1e-9) state='1';
    }
    out << state; ages.push_back(age);
  }
  out << "\",\"cell_observation_age_ms\":[";
  for (size_t i=0; i<ages.size(); ++i) { if(i)out<<','; out<<ages[i]; }
  out << "],\"surface_points_xyz\":[";
  for (size_t i=0; i<corridor_surface_points_.size(); ++i) {
    if (i) out << ',';
    const auto &p=corridor_surface_points_[i];
    out << '[' << p.x() << ',' << p.y() << ',' << p.z() << ']';
  }
  out << "],\"surface_points_complete\":" << (corridor_surface_complete_ ? "true" : "false")
      << ",\"complete\":true,\"raw_occupancy_not_robot_inflated\":true,"
      << "\"support_source\":\"same_revision_measured_surface_returns_only\","
      << "\"navigation_ground_truth_used\":false}";
  std_msgs::msg::String msg; msg.data=out.str(); corridor_snapshot_pub_->publish(msg);
}
