#include "multimap_nav_server/move_base_action_client.hpp"

namespace multimap_nav {

void MoveBaseHandler::ActivateClient(const std::string &action_name) {
  mb_client_ = std::make_unique<
      actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(
      action_name, true);
  latest_feedback = boost::make_shared<move_base_msgs::MoveBaseFeedback>();
  ROS_INFO("Waiting for move_base server to start");
  mb_client_->waitForServer();
  ROS_INFO("move_base server started!");
}

void MoveBaseHandler::SendGoal(const move_base_msgs::MoveBaseGoal &goal) {
  ROS_INFO("Sending Goal (x = %.2f, y = %.2f, z = %.2f) to move_base.",
           goal.target_pose.pose.position.x, goal.target_pose.pose.position.y,
           goal.target_pose.pose.position.z);

  mb_client_->sendGoal(
      goal, boost::bind(&MoveBaseHandler::ResultCallback, this, _1, _2),
      boost::bind(&MoveBaseHandler::ActiveCallback, this),
      boost::bind(&MoveBaseHandler::FeedbackCallback, this, _1));
}

actionlib::SimpleClientGoalState MoveBaseHandler::GetClientState() {
  return mb_client_->getState();
}

void MoveBaseHandler::CancelGoal() {
  while (mb_client_->getState() !=
         actionlib::SimpleClientGoalState::PREEMPTED) {
    mb_client_->cancelGoal();
    mb_client_->waitForResult(ros::Duration(5.0));
  }
}

void MoveBaseHandler::ActiveCallback() {
  ROS_INFO("move_base accepted the goal!");
}

void MoveBaseHandler::FeedbackCallback(
    const move_base_msgs::MoveBaseFeedbackConstPtr &feedback) {
  latest_feedback = feedback;
}

void MoveBaseHandler::ResultCallback(
    const actionlib::SimpleClientGoalState &state,
    const move_base_msgs::MoveBaseResultConstPtr &result) {
  ROS_INFO("Goal Finished with a state: %s", state.toString().c_str());
}

MoveBaseBTNode::MoveBaseBTNode(const std::string &bt_node_name,
                               const BT::NodeConfiguration &config)
    : StatefulActionNode(bt_node_name, config) {
  std::string nav_action_name;
  ros::param::get("nav_action_name", nav_action_name);
  MoveBaseHandler::GetInstance().ActivateClient(nav_action_name);
}

BT::PortsList MoveBaseBTNode::providedPorts() {
  return {BT::InputPort<move_base_msgs::MoveBaseGoal>("mb_goal")};
}

BT::NodeStatus MoveBaseBTNode::onStart() {
  move_base_msgs::MoveBaseGoal goal;
  if (!getInput<move_base_msgs::MoveBaseGoal>("mb_goal", goal)) {
    throw BT::RuntimeError("No goal set and mb_client called.");
  }
  MoveBaseHandler::GetInstance().SendGoal(goal);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveBaseBTNode::onRunning() {
  if ((MoveBaseHandler::GetInstance().GetClientState() ==
       actionlib::SimpleClientGoalState::ACTIVE) ||
      (MoveBaseHandler::GetInstance().GetClientState() ==
       actionlib::SimpleClientGoalState::PENDING)) {
    return BT::NodeStatus::RUNNING;
  } else if (MoveBaseHandler::GetInstance().GetClientState() ==
             actionlib::SimpleClientGoalState::SUCCEEDED) {
    return BT::NodeStatus::SUCCESS;
  } else {
    return BT::NodeStatus::FAILURE;
  }
}

void MoveBaseBTNode::onHalted() {}

} // namespace multimap_nav
