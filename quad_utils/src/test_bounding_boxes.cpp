#include <ros/ros.h>
#include <quad_utils/bounding_boxes.h>
#include <ros/package.h>
#include <vector>
#include <cmath> // For std::fabs

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

    // Load the YAML file path
    std::string yaml_file_path = ros::package::getPath("quad_utils") + "/config/tall_table_sizes.yaml";

    // Initialize the BoundingBoxes class
    BoundingBoxes bounding_boxes(nh, yaml_file_path);

    // Store previous bounding boxes for comparison
    std::unordered_map<std::string, BoundingBox> previous_bounding_boxes;

    ros::Rate rate(10); // Check at 10 Hz
    while (ros::ok()) {
        // Update bounding boxes
        bounding_boxes.updateBoundingBoxes();

        // Get the current bounding boxes
        auto current_bounding_boxes = bounding_boxes.getBoundingBoxes();

        // Check if any bounding box has changed
        bool has_moved = false;
        for (const auto& [link_name, current_bbox] : current_bounding_boxes) {
            if (previous_bounding_boxes.find(link_name) == previous_bounding_boxes.end() ||
                isBoundingBoxChanged(current_bbox, previous_bounding_boxes[link_name])) {
                has_moved = true;
                break;
            }
        }

        // Print only if bounding boxes have moved
        if (has_moved) {
            ROS_INFO("Bounding boxes updated:");
            for (const auto& [link_name, bbox] : current_bounding_boxes) {
                ROS_INFO_STREAM(link_name << ": "
                                          << "min_x=" << bbox.min_x << ", max_x=" << bbox.max_x
                                          << ", min_y=" << bbox.min_y << ", max_y=" << bbox.max_y
                                          << ", min_z=" << bbox.min_z << ", max_z=" << bbox.max_z);
            }
        }

        // Update previous bounding boxes
        previous_bounding_boxes = current_bounding_boxes;

        rate.sleep();
    }

    return 0;
}
