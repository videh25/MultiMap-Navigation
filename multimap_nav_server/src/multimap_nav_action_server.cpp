#include <ros/package.h>

#include "multimap_nav_server/move_base_action_client.hpp"
#include "multimap_nav_server/multimap_nav_action_server.hpp"
#include "multimap_nav_server/portal_service_client.hpp"
#include "multimap_nav_server/portal_vicinity_checker.hpp"
#include "multimap_nav_server/sql_query_maker.hpp"

namespace multimap_nav {

MultiMapNavigationServer::MultiMapNavigationServer(
    std::shared_ptr<ros::NodeHandle> nh, const std::string &action_name)
    : nh_(nh),
      as_(*nh_, action_name,
          boost::bind(&MultiMapNavigationServer::executeCB, this, _1), false),
      action_name_(action_name) {
  GenerateBehaviorTree();
  as_.start();
  ROS_INFO("Action server [%s] started.", action_name_.c_str());
}

void MultiMapNavigationServer::GenerateBehaviorTree() {
  blackboard = BT::Blackboard::create();
  std::string pkg_path = ros::package::getPath("multimap_nav_server");

  std::string start_map_id;
  ros::param::get("start_map_id", start_map_id);
  blackboard->set<std::shared_ptr<ros::NodeHandle>>("node_handle", nh_);
  blackboard->set<std::string>("current_map_id", start_map_id);

  factory_.registerNodeType<MoveBaseBTNode>("SendGoalToMoveBase");
  factory_.registerNodeType<PortalServiceBTNode>("RequestTeleportation");
  factory_.registerNodeType<PortalVicinityCheck>("CheckNearPortal");
  factory_.registerNodeType<SetCurrentPortalBTNode>("QueryLocalPortal");
  factory_.registerSimpleCondition(
      "CheckOnSameMap",
      [](BT::TreeNode &self) -> BT::NodeStatus {
        std::string goal_map_id, current_map_id;
        goal_map_id = self.getInput<std::string>("goal_map_id").value();
        current_map_id = self.getInput<std::string>("current_map_id").value();

        return (goal_map_id == current_map_id) ? BT::NodeStatus::SUCCESS
                                               : BT::NodeStatus::FAILURE;
      },
      {BT::InputPort<std::string>("goal_map_id"),
       BT::InputPort<std::string>("current_map_id")});
  factory_.registerSimpleAction(
      "SetMbGoalTo",
      [](BT::TreeNode &self) -> BT::NodeStatus {
        geometry_msgs::Pose goal_pose;
        self.getInput<geometry_msgs::Pose>("goal_pose", goal_pose);
        move_base_msgs::MoveBaseGoal mb_goal;
        mb_goal.target_pose.header.frame_id = "map";
        mb_goal.target_pose.pose = goal_pose;

        self.setOutput("mb_goal", mb_goal);

        return BT::NodeStatus::SUCCESS;
      },
      {BT::InputPort<geometry_msgs::Pose>("goal_pose"),
       BT::OutputPort<move_base_msgs::MoveBaseGoal>("mb_goal")});

  tree_ = factory_.createTreeFromFile(pkg_path + "/config/multimap_nav_bt.xml",
                                      blackboard);
}

void MultiMapNavigationServer::executeCB(
    const multimap_msgs::MultiMapNavigationGoalConstPtr &goal) {
  ROS_INFO("Received goal to map: %s and position (%.2f, %.2f)",
           goal->map_id.c_str(), goal->pose.pose.position.x,
           goal->pose.pose.position.y);

  feedback_.current_action = "";
  result_.data = false;

  blackboard->set<std::string>("goal_map_id", goal->map_id.c_str());
  blackboard->set<geometry_msgs::Pose>("final_goal_pose", goal->pose.pose);
  BT::NodeStatus status = tree_.tickRoot();

  while (ros::ok() && status == BT::NodeStatus::RUNNING) {
    feedback_.current_action = "Action BT is running...";
    as_.publishFeedback(feedback_);

    ros::Duration(0.5).sleep();
    status = tree_.tickRoot();
  }

  if (status == BT::NodeStatus::SUCCESS) {
    ROS_INFO("Multimap Navigation successfully executed");
    result_.data = true;
    as_.setSucceeded(result_);
  } else {
    ROS_ERROR("Multimap Navigation failed.");
    as_.setAborted();
  }
};

} // namespace multimap_nav
