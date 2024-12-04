#include "quad_utils/apply_force.h"
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelState.h>
#include <geometry_msgs/Wrench.h>
#include <gazebo_msgs/GetModelState.h>
#include <geometry_msgs/Point.h>
#include <thread>
#include <chrono>

void applyForceToModel(ros::NodeHandle &nh, const std::string &target_body, double force_x, double force_y, double force_z, double duration)
{
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

    // Call the service and check for success
    if (client.call(srv))
    {
        if (srv.response.success)
        {
            ROS_INFO("Force successfully applied to %s", target_body.c_str());
        }
        else
        {
            ROS_WARN("Service call succeeded but force application failed for %s", target_body.c_str());
        }
    }
    else
    {
        ROS_ERROR("Failed to call service apply_body_wrench for %s", target_body.c_str());
    }
}

void applyPeriodicForceToLegs(ros::NodeHandle &nh, const std::vector<std::string> &legs, double force_y, double duration_secs, double interval_secs)
{
    while (ros::ok())
    {
        // Apply force in positive y-direction
        for (const std::string &leg : legs)
        {
            applyForceToModel(nh, leg, 0.0, force_y, 0.0, duration_secs);
        }

        // Wait for the interval
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(interval_secs * 1000)));

        // Apply force in negative y-direction
        for (const std::string &leg : legs)
        {
            applyForceToModel(nh, leg, 0.0, -force_y, 0.0, duration_secs);
        }

        // Wait for the interval
        std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(interval_secs * 1000)));
    }
}

void applyContinuousForceToLegs(ros::NodeHandle& nh, const std::vector<std::string>& legs, double force_y, double duration_secs) {
    // ros::Rate rate(100); // Adjust the rate as needed
    // ros::Time start_time = ros::Time::now();
    // ros::Duration duration(duration_secs);

    while (ros::ok()) {
        // Apply force in positive y-direction
        for (const std::string& leg : legs) {
            applyForceToModel(nh, leg, 0.0, force_y, 0.0, 0.1); // Apply force for a short duration repeatedly
        }

        // rate.sleep();
    }
}

// Function to apply forces to move a box in a circular motion
void applyCircularMotion(ros::NodeHandle &nh, const std::vector<std::string> &legs, double radius, double angular_velocity, double duration_secs, double interval_secs)
{
    double angle = 0.0;
    ros::Rate rate(1.0 / interval_secs);
    while (ros::ok())
    {
        double force_x = radius * angular_velocity * std::cos(angle);
        double force_y = radius * angular_velocity * std::sin(angle);

        for (const std::string &leg : legs)
        {
            applyForceToModel(nh, leg, force_x, force_y, 0.0, duration_secs);
        }

        angle += angular_velocity * interval_secs;
        if (angle >= 2 * M_PI)
        {
            angle -= 2 * M_PI;
        }

        rate.sleep();
    }
}

bool getInitialPosition(ros::NodeHandle& nh, const std::string& model_name, double& x, double& y, double& z) {
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    gazebo_msgs::GetModelState srv;

    srv.request.model_name = model_name;

    if (client.call(srv)) {
        if (srv.response.success) {
            x = srv.response.pose.position.x;
            y = srv.response.pose.position.y;
            z = srv.response.pose.position.z;
            ROS_INFO("Initial position of %s: x=%f, y=%f, z=%f", model_name.c_str(), x, y, z);
            return true;
        } else {
            ROS_WARN("Service call succeeded but failed to get initial position for %s", model_name.c_str());
        }
    } else {
        ROS_ERROR("Failed to call service get_model_state for %s", model_name.c_str());
    }
    return false;
}

void setPosition(ros::NodeHandle& nh, const std::string& model_name, double x, double y, double z) {
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gazebo_msgs::SetModelState srv;

    gazebo_msgs::ModelState model_state;
    model_state.model_name = model_name;
    model_state.pose.position.x = x;
    model_state.pose.position.y = y;
    model_state.pose.position.z = z;
    model_state.pose.orientation.w = 1.0; // No rotation
    model_state.pose.orientation.x = 0.0;
    model_state.pose.orientation.y = 0.0;
    model_state.pose.orientation.z = 0.0;

    srv.request.model_state = model_state;

    // Call the service and check for success
    if (client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("Position successfully set for %s", model_name.c_str());
        } else {
            ROS_WARN("Service call succeeded but position setting failed for %s", model_name.c_str());
        }
    } else {
        ROS_ERROR("Failed to call service set_model_state for %s", model_name.c_str());
    }
}

void moveBlockContinuously(ros::NodeHandle& nh, const std::string& model_name, double velocity_x, double velocity_y, double velocity_z, double duration_secs) {
    double x, y, z;
    if (!getInitialPosition(nh, model_name, x, y, z)) {
        ROS_ERROR("Failed to get initial position. Exiting.");
        return;
    }

    ros::Rate rate(100); // Adjust the rate as needed
    ros::Time start_time = ros::Time::now();
    ros::Duration duration(duration_secs);

    while (ros::ok() && (ros::Time::now() - start_time) < duration) {
        x += velocity_x / 100.0; // Update position based on velocity and rate
        // y += velocity_y / 100.0;
        // z += velocity_z / 100.0;

        setPosition(nh, model_name, x, y, z);

        rate.sleep();
    }
}
