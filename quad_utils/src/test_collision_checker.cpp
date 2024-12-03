#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <quad_utils/collision_checker.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "collision_checker_test_node");
    ros::NodeHandle nh;

    // Initialize the CollisionChecker object
    CollisionChecker collision_checker(nh);

    // Wait for the bounding boxes to be received
    ros::Duration(1.0).sleep();

    // Define a point to check for collision
    geometry_msgs::Point point;
    point.x = 2.1;
    point.y = -0.25;
    point.z = 0.36;

    // Process incoming messages
    ros::spinOnce();

    // Check for collision
    if (collision_checker.isInCollision(point)) {
        ROS_WARN("Collision detected!");
    } else {
        ROS_INFO("No collision detected.");
    }

    return 0;
}