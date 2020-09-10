#include <ros/ros.h>
#include "package_template/package_template.h"

#include <yaml-cpp/yaml.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "package_template_node");
	ros::NodeHandle nh;

	PackageTemplate package_template;

	ros::Rate r(100); // Update rate
	while (ros::ok()) {
		package_template.update();
		r.sleep();
	}
	return 0;
}