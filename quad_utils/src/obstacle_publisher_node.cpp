#include "quad_utils/obstacle_publisher.h"

int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "obstacle_publisher_node");
    ros::NodeHandle nh;

    if (argc < 2) {
        ROS_ERROR("Usage: rosrun quad_utils obstacle_publisher_node <link_name>");
        return 1;
    }

    std::string link_name = argv[1];
    ObstaclePublisher publisher(nh, link_name);

    ros::Rate rate(10); // 10 Hz
    while (ros::ok()) {
        publisher.publishPosition();
        rate.sleep();
    }

    return 0;
}
