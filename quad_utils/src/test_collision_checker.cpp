#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <quad_utils/collision_checker.h>
#include <quad_utils/bounding_boxes.h>
#include <ros/package.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_collision_checker_node");
    ros::NodeHandle nh;

    // Initialize the BoundingBoxes object
    std::string yaml_file = ros::package::getPath("quad_utils") + "/config/box_sizes.yaml";
    BoundingBoxes bounding_boxes(nh, yaml_file);

    // Initialize the CollisionChecker object
    CollisionChecker collision_checker(nh);

    // Wait for the bounding boxes to be received
    ros::Duration(1.0).sleep();

    // Define a point to check for collision
    geometry_msgs::Point point;
    point.x = 1.0;
    point.y = 0.0;
    point.z = 0.5;

    // Check for collision
    if (collision_checker.isInCollision(point)) {
        ROS_WARN("Collision detected!");
    } else {
        ROS_INFO("No collision detected.");
    }

    return 0;
}