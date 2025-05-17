#ifndef ___PORTAL_VICINITY_CHECKER___
#define ___PORTAL_VICINITY_CHECKER___

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "ros/ros.h"

class PortalVicinityCheck : public BT::SyncActionNode {
public:
  PortalVicinityCheck(const std::string &name,
                      const BT::NodeConfiguration &config);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;

  void AmclPoseCallback(
      const geometry_msgs::PoseWithCovarianceStampedPtr &latest_pose);

protected:
  ros::Subscriber amcl_pose_sub_;
  geometry_msgs::PoseWithCovarianceStampedPtr latest_pose_;
  std::mutex latest_pose_mtx;
};

#endif
