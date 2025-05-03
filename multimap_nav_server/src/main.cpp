#include <ros/ros.h>

#include "multimap_nav_server/multimap_nav_action_server.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "multimap_navigation_server");
  std::shared_ptr<ros::NodeHandle> nh_ = std::make_shared<ros::NodeHandle>();
  multimap_nav::MultiMapNavigationServer server(nh_, "multimap_navigation");
  ros::AsyncSpinner s_(3);
  s_.start();

  ros::waitForShutdown();
  s_.stop();
  return 0;
}
