#include "quad_utils/bounding_boxes.h"
#include <ros/ros.h>
#include <ros/package.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "bboxes_node");
    ros::NodeHandle nh;

    std::string yaml_file = ros::package::getPath("quad_utils") + "/config/minibox_sizes.yaml";
    BoundingBoxes bounding_boxes(nh, yaml_file);

    ros::spin(); // Keep the node running to process callbacks

    return 0;
}