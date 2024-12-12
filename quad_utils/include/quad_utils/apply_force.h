#ifndef APPLY_FORCE_H
#define APPLY_FORCE_H

#include <ros/ros.h>
#include <string>
#include <vector>

// Global variable declaration
extern int object_direction;

// Function to apply force to a specific body of the model
void applyForceToModel(ros::NodeHandle& nh, const std::string& target_body, double force_x, double force_y, double force_z, double duration = 0.5);

void applyContinuousForceToLegs(ros::NodeHandle& nh, const std::vector<std::string>& legs, double force_y, double duration_secs);


// Function to apply periodic force to multiple legs of the table
void applyPeriodicForceToLegs(ros::NodeHandle& nh, const std::vector<std::string>& legs, double force_y, double duration_secs, double interval_secs);

void moveBlockContinuously(ros::NodeHandle& nh, const std::string& model_name, double velocity_x, double velocity_y, double velocity_z, double duration_secs, double period_secs);
void setPosition(ros::NodeHandle& nh, const std::string& model_name, double x, double y, double z);
bool getInitialPosition(ros::NodeHandle& nh, const std::string& model_name, double& x, double& y, double& z);

#endif // APPLY_FORCE_H
