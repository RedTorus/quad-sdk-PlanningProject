#include <ros/ros.h>
#include "ros/package.h"
#include "global_body_planner/global_body_planner.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "global_body_planner");
  ros::NodeHandle nh;

  std::string yaml_file_path = ros::package::getPath("quad_utils") + "/config/short_table_sizes.yaml";
  planning_utils::initializeCollisionChecker(nh, yaml_file_path);

  GlobalBodyPlanner global_body_planner(nh);
  global_body_planner.spin();
  return 0;
}
