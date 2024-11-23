#include <quad_utils/collision_checker.h>
#include <quad_utils/bounding_boxes.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Point.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_collision_checker_node");
    ros::NodeHandle nh;

    // Load the YAML file path
    std::string yaml_file_path = ros::package::getPath("quad_utils") + "/config/tall_table_sizes.yaml";

    // Initialize the BoundingBoxes class
    BoundingBoxes bounding_boxes(nh, yaml_file_path);

    // Initialize the CollisionChecker class
    CollisionChecker collision_checker(bounding_boxes);

    // Example test points
    geometry_msgs::Point test_point_inside;
    test_point_inside.x = 1.5;
    test_point_inside.y = 0.35;
    test_point_inside.z = 0.455;

    geometry_msgs::Point test_point_outside;
    test_point_outside.x = 1.5;
    test_point_outside.y = 0.35;
    test_point_outside.z = 0.45;

    ros::Rate rate(1); // Test at 1 Hz
    while (ros::ok()) {
        // Update bounding boxes
        bounding_boxes.updateBoundingBoxes();

        // Check for collision with the first point
        if (collision_checker.isInCollision(test_point_inside)) {
            ROS_INFO("Test point inside is in collision.");
        } else {
            ROS_INFO("Test point inside is NOT in collision.");
        }

        // Check for collision with the second point
        if (collision_checker.isInCollision(test_point_outside)) {
            ROS_INFO("Test point outside is in collision.");
        } else {
            ROS_INFO("Test point outside is NOT in collision.");
        }

        rate.sleep();
    }

    return 0;
}
