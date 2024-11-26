#ifndef OBSTACLE_PUBLISHER_H
#define OBSTACLE_PUBLISHER_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <gazebo_msgs/GetLinkState.h>

class ObstaclePublisher {
public:
    ObstaclePublisher(ros::NodeHandle& nh, const std::string& link_name);
    void publishPosition();

private:
    ros::NodeHandle nh_;
    ros::Publisher position_pub_; // Match the variable name here
    ros::ServiceClient link_state_client_;
    std::string link_name_;
};

#endif // OBSTACLE_PUBLISHER_H
