#ifndef ___MULTIMAP_NAV_HANDLER___
#define ___MULTIMAP_NAV_HANDLER___

#include "actionlib/server/simple_action_server.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "multimap_msgs/MultiMapNavigationAction.h"
#include "ros/ros.h"
#include <behaviortree_cpp_v3/bt_factory.h>

#include "multimap_nav_server/sql_query_maker.hpp"

namespace multimap_nav {

class MultiMapNavigationServer {
public:
  explicit MultiMapNavigationServer(std::shared_ptr<ros::NodeHandle> nh,
                                    const std::string &action_name);

  void GenerateBehaviorTree();

private:
  void executeCB(const multimap_msgs::MultiMapNavigationGoalConstPtr &goal);

  std::shared_ptr<ros::NodeHandle> nh_;
  actionlib::SimpleActionServer<multimap_msgs::MultiMapNavigationAction> as_;
  std::string action_name_;

  multimap_msgs::MultiMapNavigationFeedback feedback_;
  multimap_msgs::MultiMapNavigationResult result_;

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  BT::Blackboard::Ptr blackboard;
};

} // namespace multimap_nav

#endif
