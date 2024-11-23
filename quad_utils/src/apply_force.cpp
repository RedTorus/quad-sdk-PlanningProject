#include "quad_utils/apply_force.h"
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Point.h>
#include <thread>
#include <chrono>

void applyForceToModel(ros::NodeHandle& nh, const std::string& target_body, double force_x, double force_y, double force_z, double duration) {
    ros::service::waitForService("/gazebo/apply_body_wrench");
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

    gazebo_msgs::ApplyBodyWrench srv;

    // Define the force to be applied
    geometry_msgs::Wrench wrench;
    wrench.force.x = force_x;
    wrench.force.y = force_y;
    wrench.force.z = force_z;
    wrench.torque.x = 0.0;
    wrench.torque.y = 0.0;
    wrench.torque.z = 0.0;

    // Define the point where the force is applied (relative to the body frame)
    geometry_msgs::Point reference_point;
    reference_point.x = 0.0;
    reference_point.y = 0.0;
    reference_point.z = 0.0;

    // Fill in the service request
    srv.request.body_name = target_body;
    srv.request.reference_frame = "world";
    srv.request.reference_point = reference_point;
    srv.request.wrench = wrench;
    srv.request.start_time = ros::Time(0);
    srv.request.duration = ros::Duration(duration);

    if (client.call(srv)) {
        ROS_INFO("Force applied to %s: %s", target_body.c_str(), srv.response.success ? "success" : "failure");
    } else {
        ROS_ERROR("Failed to call service apply_body_wrench for %s", target_body.c_str());
    }
}

void applyPeriodicForceToLegs(ros::NodeHandle& nh, const std::vector<std::string>& legs, double force_y, double duration_secs, double interval_secs) {
    while (ros::ok()) {
        // Apply force in positive y-direction
        for (const std::string& leg : legs) {
            applyForceToModel(nh, leg, 0.0, force_y, 0.0, duration_secs);
        }

        // Wait for the interval
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(interval_secs * 1000)));

        // Apply force in negative y-direction
        for (const std::string& leg : legs) {
            applyForceToModel(nh, leg, 0.0, -force_y, 0.0, duration_secs);
        }

        // Wait for the interval
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(interval_secs * 1000)));
    }
}
