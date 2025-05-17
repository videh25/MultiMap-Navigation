#ifndef ___PORTAL_SERVICE_CLIENT___
#define ___PORTAL_SERVICE_CLIENT___

#include <cstdlib>
#include <sqlite3.h>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/PoseStamped.h"
#include "multimap_msgs/RequestTeleportation.h"
#include "ros/ros.h"

namespace multimap_nav {

class PortalServiceClient {
public:
  PortalServiceClient() {};
  void ActivateClient(const std::shared_ptr<ros::NodeHandle> &nh_,
                      const std::string &service_name);

  bool CallService(std::string goal_map_id);

protected:
  ros::ServiceClient portal_client;
};

class PortalServiceBTNode : public BT::SyncActionNode,
                            public PortalServiceClient {
public:
  PortalServiceBTNode(const std::string &name,
                      const BT::NodeConfiguration &config);
  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace multimap_nav

#endif
