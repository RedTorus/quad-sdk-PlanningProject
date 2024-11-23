#ifndef APPLY_FORCE_H
#define APPLY_FORCE_H

#include <ros/ros.h>
#include <string>
#include <vector>

// Function to apply force to a specific body of the model
void applyForceToModel(ros::NodeHandle& nh, const std::string& target_body, double force_x, double force_y, double force_z, double duration = 0.5);

// Function to apply periodic force to multiple legs of the table
void applyPeriodicForceToLegs(ros::NodeHandle& nh, const std::vector<std::string>& legs, double force_y, double duration_secs, double interval_secs);

#endif // APPLY_FORCE_H
