#include <ros/ros.h>
#include <quad_utils/bounding_boxes.h>
#include <ros/package.h>
#include <vector>
#include <cmath> // For std::fabs
#include <unordered_map>

bool isBoundingBoxChanged(const BoundingBox& current, const BoundingBox& previous) {
    const double tolerance = 1e-3; // Small threshold for floating-point comparison
    return std::fabs(current.min_x - previous.min_x) > tolerance ||
           std::fabs(current.max_x - previous.max_x) > tolerance ||
           std::fabs(current.min_y - previous.min_y) > tolerance ||
           std::fabs(current.max_y - previous.max_y) > tolerance ||
           std::fabs(current.min_z - previous.min_z) > tolerance ||
           std::fabs(current.max_z - previous.max_z) > tolerance;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_bounding_boxes_node");
    ros::NodeHandle nh;

    // Determine which YAML file to use based on the argument
    std::string table_type = "box"; // Default to short
    if (argc > 1) {
        table_type = argv[1];
    }

    std::string yaml_file_path;
    if (table_type == "tall") {
        yaml_file_path = ros::package::getPath("quad_utils") + "/config/tall_table_sizes.yaml";
    } else if (table_type == "short") {
        yaml_file_path = ros::package::getPath("quad_utils") + "/config/short_table_sizes.yaml";
    }
    else{
        yaml_file_path = ros::package::getPath("quad_utils") + "/config/box_sizes.yaml";
    }

    // Initialize the BoundingBoxes class
    BoundingBoxes bounding_boxes(nh, yaml_file_path);

    // Store previous bounding boxes for comparison
    std::unordered_map<std::string, BoundingBox> previous_bounding_boxes;

    ros::Rate rate(0.5); // Check at 10 Hz
    while (ros::ok()) {
        // Update bounding boxes
        bounding_boxes.updateBoundingBoxes();

        // Get the current bounding boxes
        auto current_bounding_boxes = bounding_boxes.getBoundingBoxes();

        // Compare current and previous bounding boxes
        for (const auto& pair : current_bounding_boxes) {
            const std::string& name = pair.first;
            const BoundingBox& current = pair.second;
            ROS_INFO("Bounding box for %s: min_x=%f, max_x=%f, min_y=%f, max_y=%f, min_z=%f, max_z=%f",
                     name.c_str(), current.min_x, current.max_x, current.min_y, current.max_y, current.min_z, current.max_z);
            if (previous_bounding_boxes.find(name) != previous_bounding_boxes.end()) {
                const BoundingBox& previous = previous_bounding_boxes[name];
                if (isBoundingBoxChanged(current, previous)) {
                    ROS_INFO("Bounding box for %s has changed.", name.c_str());
                }
            }

            // Update the previous bounding box
            previous_bounding_boxes[name] = current;
        }

        rate.sleep();
    }

    return 0;
}