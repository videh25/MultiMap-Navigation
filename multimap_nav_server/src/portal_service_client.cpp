#include "multimap_nav_server/portal_service_client.hpp"

namespace multimap_nav {

void PortalServiceClient::ActivateClient(
    const std::shared_ptr<ros::NodeHandle> &nh_,
    const std::string &service_name) {
  portal_client =
      nh_->serviceClient<multimap_msgs::RequestTeleportation>(service_name);
}

bool PortalServiceClient::CallService(std::string goal_map_id) {
  multimap_msgs::RequestTeleportation srv;
  srv.request.goal_map_id = goal_map_id;
  ROS_INFO("Requesting teleportation to %s", goal_map_id.c_str());
  if (!portal_client.call(srv)) {
    ROS_INFO("Teleportation Failed.");
    return false;
  }
  ROS_INFO("Teleportation Successful.");
  return true;
}

PortalServiceBTNode::PortalServiceBTNode(const std::string &name,
                                         const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config) {
  std::string service_name;
  std::shared_ptr<ros::NodeHandle> nh_;
  ros::param::get("teleportation_service", service_name);
  getInput("node_handle", nh_);

  ActivateClient(nh_, service_name);
}

BT::PortsList PortalServiceBTNode::providedPorts() {
  return {BT::InputPort<std::shared_ptr<ros::NodeHandle>>("node_handle"),
          BT::InputPort<std::string>("goal_map_id"),
          BT::OutputPort<std::string>("current_map_id")};
}

BT::NodeStatus PortalServiceBTNode::tick() {
  std::string goal_map_id;
  getInput("goal_map_id", goal_map_id);

  if (!CallService(goal_map_id)) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("current_map_id", goal_map_id);
  return BT::NodeStatus::SUCCESS;
}

} // namespace multimap_nav
