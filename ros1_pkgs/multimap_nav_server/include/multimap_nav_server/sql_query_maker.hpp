#ifndef ___SQL_QUERY_HANDLER___
#define ___SQL_QUERY_HANDLER___

#include <cstdlib>
#include <sqlite3.h>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/Pose.h"
#include "move_base_msgs/MoveBaseGoal.h"
#include "ros/ros.h"

namespace multimap_nav {

class SqlQueryMaker {
public:
  SqlQueryMaker() {};
  ~SqlQueryMaker();

  void SetDatabasePath(std::string database_path);
  int UpdatePoseToWormholeLocation(geometry_msgs::PosePtr goal_pose,
                                   std::string map_id);

protected:
  sqlite3 *db;        // Database connection pointer
  sqlite3_stmt *stmt; // Statement pointersqlite3* db;
};

class SetCurrentPortalBTNode : public BT::SyncActionNode, SqlQueryMaker {
public:
  SetCurrentPortalBTNode(const std::string &name,
                         const BT::NodeConfiguration &config);
  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

} // namespace multimap_nav

#endif
