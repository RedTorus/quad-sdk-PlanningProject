#include "quad_utils/bounding_boxes.h"
#include <ros/ros.h>
#include <ros/package.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "bboxes_node");
    ros::NodeHandle nh;

    std::string yaml_file = ros::package::getPath("quad_utils") + "/config/box_sizes.yaml";
    BoundingBoxes bounding_boxes(nh, yaml_file);

    ros::Rate rate(1.0); // Publish at 1 Hz
    while (ros::ok()) {
        bounding_boxes.updateBoundingBoxes();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}