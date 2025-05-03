#ifndef ___MOVE_BASE_CLIENT_BEHAVIOR_TREE_NODE___
#define ___MOVE_BASE_CLIENT_BEHAVIOR_TREE_NODE___

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "multimap_msgs/MultiMapNavigationFeedback.h"
#include "ros/ros.h"

namespace multimap_nav {

// Class to call and execute move base goal action
class MoveBaseHandler {
public:
  // Singleton
  MoveBaseHandler(const MoveBaseHandler &) = delete;
  MoveBaseHandler &operator=(const MoveBaseHandler &) = delete;
  MoveBaseHandler(MoveBaseHandler &&) = delete;
  MoveBaseHandler &operator=(MoveBaseHandler &&) = delete;

  void ActivateClient(const std::string &action_name);
  static MoveBaseHandler &GetInstance() {
    static MoveBaseHandler
        instance; // Guaranteed to be initialized once in a thread-safe manner
    return instance;
  };

  void SendGoal(const move_base_msgs::MoveBaseGoal &goal);
  actionlib::SimpleClientGoalState GetClientState();
  void CancelGoal();

protected:
  void ActiveCallback();
  void
  FeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback);
  void ResultCallback(const actionlib::SimpleClientGoalState &state,
                      const move_base_msgs::MoveBaseResultConstPtr &result);
  std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>
      mb_client_;

  move_base_msgs::MoveBaseFeedbackConstPtr latest_feedback;

private:
  MoveBaseHandler() {};
  ~MoveBaseHandler() {};
  static MoveBaseHandler instance;
};

class MoveBaseBTNode : public BT::StatefulActionNode {
public:
  MoveBaseBTNode(const std::string &name, const BT::NodeConfiguration &config);
  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

} // namespace multimap_nav

#endif
