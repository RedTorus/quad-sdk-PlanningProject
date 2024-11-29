#include "quad_utils/obstacle_sdf_name_publisher.h"
#include <ros/package.h>
// Constructor implementation
BoxSdfPublisher::BoxSdfPublisher(ros::NodeHandle& nh) {
    // Initialize the publisher
    sdf_publisher_ = nh.advertise<std_msgs::String>("box_description", 10);

    // Fetch the box_sdf parameter
    if (!nh.getParam("box_sdf", box_sdf_)) {
        ROS_ERROR("Failed to get param 'box_sdf'. Make sure it is set in the launch file.");
        ros::shutdown();
    }
}

// Determine the message based on box_sdf
std::string BoxSdfPublisher::determineMessage(const std::string& box_sdf) {
    std::string yaml_file_path; // Declare it here

    if (box_sdf.find("table_short.sdf") != std::string::npos) {
        yaml_file_path = ros::package::getPath("quad_utils") + "/config/short_table_sizes.yaml";//"/config/box_sizes.yaml";
    } else if (box_sdf.find("table_tall.sdf") != std::string::npos) {
        yaml_file_path = ros::package::getPath("quad_utils") + "/config/tall_table_sizes.yaml";//"/config/box_sizes.yaml";
    } else if (box_sdf.find("table_longshort.sdf") != std::string::npos) {
        yaml_file_path = ros::package::getPath("quad_utils") + "/config/longshort_table_sizes.yaml";//"/config/box_sizes.yaml";
    } else if (box_sdf.find("box.sdf") != std::string::npos) {
        yaml_file_path = ros::package::getPath("quad_utils") + "/config/box_sizes.yaml";//"/config/box_sizes.yaml";
    } else if (box_sdf.find("minibox.sdf") != std::string::npos) {
        yaml_file_path = ros::package::getPath("quad_utils") + "/config/minibox_sizes.yaml";//"/config/box_sizes.yaml";
    } else {
        yaml_file_path = ros::package::getPath("quad_utils") + "/config/box_sizes.yaml";//"/config/box_sizes.yaml";
    }

    return yaml_file_path;
}

// Run method to publish the message
void BoxSdfPublisher::run() {
    //ros::Rate loop_rate(0.0001); // 1 Hz
    std_msgs::String msg;
    msg.data = determineMessage(box_sdf_);
    std::string param_name = "/yaml";
    std::string param_val = msg.data;
    //ROS_INFO("Setting param %s to %s", param_name.c_str(), param_val.c_str());
    ros::param::set(param_name, param_val);

    /* while (ros::ok()) {
        sdf_publisher_.publish(msg);
        ROS_INFO("Published: %s", msg.data.c_str());
        ros::spinOnce();
        loop_rate.sleep();
    } */
}
