#include "quad_utils/obstacle_publisher.h"

ObstaclePublisher::ObstaclePublisher(ros::NodeHandle& nh, const std::string& link_name)
    : nh_(nh), link_name_(link_name) {
    // Publisher for obstacle position
    position_pub_ = nh_.advertise<geometry_msgs::Point>("obstacle_position", 10);

    // Service client to get link state
    link_state_client_ = nh_.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
}

void ObstaclePublisher::publishPosition() {
    ROS_INFO("Attempting to call /gazebo/get_link_state for link: %s", link_name_.c_str());
    gazebo_msgs::GetLinkState link_state_srv;
    link_state_srv.request.link_name = link_name_;
    link_state_srv.request.reference_frame = "world"; // Specify the global frame

    if (link_state_client_.call(link_state_srv)) {
        const auto& position = link_state_srv.response.link_state.pose.position;
        ROS_INFO("Global Link position: x=%f, y=%f, z=%f", position.x, position.y, position.z);
        
        geometry_msgs::Point point_msg;
        point_msg.x = position.x;
        point_msg.y = position.y;
        point_msg.z = position.z;

        position_pub_.publish(point_msg);
        ROS_INFO("Published global position.");
    } else {
        ROS_WARN("Failed to call service /gazebo/get_link_state for %s", link_name_.c_str());
    }
}

