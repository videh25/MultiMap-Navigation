#include <math.h>
#include <mutex>

#include "multimap_nav_server/portal_vicinity_checker.hpp"

PortalVicinityCheck::PortalVicinityCheck(const std::string &name,
                                         const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config) {
  std::string pose_topic;
  ros::param::get("pose_topic", pose_topic);

  std::shared_ptr<ros::NodeHandle> nh_;
  getInput("node_handle", nh_);

  amcl_pose_sub_ = nh_->subscribe(pose_topic, 1,
                                  &PortalVicinityCheck::AmclPoseCallback, this);
}

void PortalVicinityCheck::AmclPoseCallback(
    const geometry_msgs::PoseWithCovarianceStampedPtr &latest_pose) {
  std::lock_guard<std::mutex> lc_gd(latest_pose_mtx);
  latest_pose_ = latest_pose;
}

BT::PortsList PortalVicinityCheck::providedPorts() {
  return {BT::InputPort<std::shared_ptr<ros::NodeHandle>>("node_handle"),
          BT::InputPort<geometry_msgs::Pose>("current_map_portal_pose")};
}

BT::NodeStatus PortalVicinityCheck::tick() {
  geometry_msgs::Pose current_map_portal_pose;
  getInput("current_map_portal_pose", current_map_portal_pose);

  std::lock_guard<std::mutex> lc_gd(latest_pose_mtx);
  float distance = std::hypotf(
      (current_map_portal_pose.position.x - latest_pose_->pose.pose.position.x),
      (current_map_portal_pose.position.y -
       latest_pose_->pose.pose.position.y));

  if (distance > 0.5) {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::SUCCESS;
}
