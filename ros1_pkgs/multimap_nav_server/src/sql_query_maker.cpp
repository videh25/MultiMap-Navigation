#include "multimap_nav_server/sql_query_maker.hpp"

namespace multimap_nav {

void SqlQueryMaker::SetDatabasePath(std::string database_path) {
  // Open the SQLite database
  if (sqlite3_open(database_path.c_str(), &db) != SQLITE_OK) {
    ROS_ERROR_STREAM("Error opening database: " << sqlite3_errmsg(db)
                                                << std::endl);
    exit(1);
  }

  // Define the query statement
  const char *query =
      "SELECT x, y, qx, qy, qz, qw FROM map_data WHERE map_id = ?";
  if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK) {
    ROS_ERROR_STREAM("Error preparing statement: " << sqlite3_errmsg(db)
                                                   << std::endl);
    sqlite3_close(db);
    exit(1);
  }
  // const char* path_cstr = std::getenv("MULTIMAP_STORAGE_PATH");

  // if (path_cstr == nullptr) {
  //     ROS_ERROR("Environment variable MULTIMAP_STORAGE_PATH is not set.");
  //     exit(1);
  // }
  // std::string path(path_cstr);
  // std::string database_path = path + database_name;
}

int SqlQueryMaker::UpdatePoseToWormholeLocation(
    geometry_msgs::PosePtr goal_pose, std::string map_id) {
  // Bind the map_id to the statement
  if (sqlite3_bind_text(stmt, 1, map_id.c_str(), -1, SQLITE_STATIC) !=
      SQLITE_OK) {
    ROS_INFO_STREAM("Error binding value: " << sqlite3_errmsg(db) << std::endl);
    sqlite3_finalize(stmt);
    sqlite3_close(db);
    return 1;
  }

  int rc = sqlite3_step(stmt);
  float x, y, qx, qy, qz, qw;
  if (rc == SQLITE_ROW) {
    // Successfully retrieved data
    x = sqlite3_column_double(stmt, 0);
    y = sqlite3_column_double(stmt, 1);
    qx = sqlite3_column_double(stmt, 2);
    qy = sqlite3_column_double(stmt, 3);
    qz = sqlite3_column_double(stmt, 4);
    qw = sqlite3_column_double(stmt, 5);

    ROS_INFO_STREAM("SQL Retrieved WORMHOLE LOCATIONS\nMap ID: " << map_id
                                                                 << std::endl);
    ROS_INFO_STREAM("X: " << x << ", Y: " << y << ", qx " << qx << ", qy " << qy
                          << ", qz " << qz << ", qw " << qw << std::endl);

  } else if (rc == SQLITE_DONE) {
    ROS_ERROR_STREAM("No data found for map_id: " << map_id << std::endl);
  } else {
    ROS_ERROR_STREAM("Error executing query: " << sqlite3_errmsg(db)
                                               << std::endl);
  }

  sqlite3_reset(stmt);
  sqlite3_clear_bindings(stmt);

  goal_pose->position.x = x;
  goal_pose->position.y = y;
  goal_pose->position.z = 0.0;
  goal_pose->orientation.x = qx;
  goal_pose->orientation.y = qy;
  goal_pose->orientation.z = qz;
  goal_pose->orientation.w = qw;

  return 0;
}

SqlQueryMaker::~SqlQueryMaker() {
  sqlite3_finalize(stmt);
  sqlite3_close(db);
}

SetCurrentPortalBTNode::SetCurrentPortalBTNode(
    const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config) {
  std::string database_path;
  ros::param::get("database_path", database_path);
  SetDatabasePath(database_path);
}

BT::PortsList SetCurrentPortalBTNode::providedPorts() {
  return {BT::OutputPort<geometry_msgs::Pose>("current_map_portal_pose"),
          BT::InputPort<std::string>("current_map_id")};
}

BT::NodeStatus SetCurrentPortalBTNode::tick() {
  geometry_msgs::PosePtr goal_pose = boost::make_shared<geometry_msgs::Pose>();
  std::string current_map_id;
  getInput<std::string>("current_map_id", current_map_id);

  if (UpdatePoseToWormholeLocation(goal_pose, current_map_id) != 0) {
    return BT::NodeStatus::FAILURE;
  }

  setOutput("current_map_portal_pose", *goal_pose);
  return BT::NodeStatus::SUCCESS;
}

} // namespace multimap_nav
