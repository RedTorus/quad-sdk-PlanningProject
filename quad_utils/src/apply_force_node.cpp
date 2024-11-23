/* #include "apply_force.cpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "apply_force_node");
    ros::NodeHandle nh;

    // Call the function in apply_force.cpp to apply the force
    applyForceToModel(nh);

    ros::spin();  // Keep the node running for additional actions if required
    return 0;
}
 */


/* #include "quad_utils/apply_force.h"
#include <ros/ros.h>
#include <vector>

void applyPeriodicForceToLegs(ros::NodeHandle& nh, const std::vector<std::string>& legs, double force_y, double duration_secs, double interval_secs) {
    bool apply_positive_force = true;

    ros::Rate loop_rate(1.0 / interval_secs); // Control frequency of force application
    while (ros::ok()) {
        double current_force_y = apply_positive_force ? force_y : -force_y;

        for (const auto& leg : legs) {
            ROS_INFO("Applying force on %s: %f in y-direction", leg.c_str(), current_force_y);
            applyForceToModel(nh, leg, 0.0, current_force_y, 0.0, duration_secs);
        }

        apply_positive_force = !apply_positive_force; // Alternate the direction of force
        loop_rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "apply_force_node");
    ros::NodeHandle nh;

    // Define the legs of the table
    std::vector<std::string> legs = {
        "box::leg1", // Replace with the actual names of the legs in your Gazebo model
        "box::leg2",
        "box::leg3",
        "box::leg4"
    };

    // Set the force parameters
    double force_y = 380.0;                     // Magnitude of the force in y-direction
    double duration_secs = 0.5;                 // Duration for each force application
    double interval_secs = 1.0;                 // Interval between alternating forces

    // Apply periodic forces to the legs
    applyPeriodicForceToLegs(nh, legs, force_y, duration_secs, interval_secs);

    return 0;
}

 */

#include "quad_utils/apply_force.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "apply_force_node");
    ros::NodeHandle nh;

    std::vector<std::string> legs = {"box::leg1", "box::leg2", "box::leg3", "box::leg4"};
    double force_y = 380.0;   // Force magnitude
    double duration = 0.5;   // Duration of each force application
    double interval = 1.0;   // Interval between force reversals

    // Apply periodic forces
    applyPeriodicForceToLegs(nh, legs, force_y, duration, interval);

    return 0;
}


