#ifndef OBSTACLE_SDF_NAME_PUBLISHER_H
#define OBSTACLE_SDF_NAME_PUBLISHER_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>

class BoxSdfPublisher {
public:
    // Constructor
    BoxSdfPublisher(ros::NodeHandle& nh);

    // Method to run the publisher logic
    void run();

private:
    // Helper function to determine the string to publish based on box_sdf
    std::string determineMessage(const std::string& box_sdf);

    // ROS publisher
    ros::Publisher sdf_publisher_;

    // Parameter storing the box_sdf value
    std::string box_sdf_;
};

#endif // BOX_SDF_PUBLISHER_H
