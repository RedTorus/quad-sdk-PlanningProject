#ifndef BOUNDING_BOXES_H
#define BOUNDING_BOXES_H

#include <ros/ros.h>
#include <gazebo_msgs/GetLinkState.h>
#include <string>
#include <unordered_map>
#include <vector>

// Define a structure for link sizes
struct LinkSize {
    std::string link_name;
    double length;
    double width;
    double height;
};

// Define a structure for bounding boxes
struct BoundingBox {
    double min_x;
    double max_x;
    double min_y;
    double max_y;
    double min_z;
    double max_z;
};

// Class for managing bounding boxes
class BoundingBoxes {
public:
    BoundingBoxes(ros::NodeHandle& nh, const std::string& yaml_file);

    void updateBoundingBoxes();
    const std::unordered_map<std::string, BoundingBox>& getBoundingBoxes() const;

private:
    BoundingBox computeBoundingBox(const gazebo_msgs::LinkState& link_state, const LinkSize& size);
    void loadLinkSizes(const std::string& yaml_file);

    ros::NodeHandle nh_;
    ros::ServiceClient link_state_client_;
    std::vector<LinkSize> link_sizes_;
    std::unordered_map<std::string, BoundingBox> bounding_boxes_;
};

#endif // BOUNDING_BOXES_H
