#include "quad_utils/bounding_boxes.h"
#include <yaml-cpp/yaml.h>

BoundingBoxes::BoundingBoxes(ros::NodeHandle& nh, const std::string& yaml_file) : nh_(nh) {
    model_states_sub_ = nh_.subscribe("/gazebo/model_states", 10, &BoundingBoxes::modelStatesCallback, this);
    bbox_pub_ = nh_.advertise<quad_msgs::BoundingBoxArray>("bounding_boxes", 10); // Initialize the publisher
    loadLinkSizes(yaml_file);
    ROS_INFO("BoundingBoxes initialized");

}

void BoundingBoxes::loadLinkSizes(const std::string& yaml_file) {
    YAML::Node config = YAML::LoadFile(yaml_file);
    for (const auto& node : config) {
        LinkSize link_size;
        link_size.link_name = node.first.as<std::string>();
        link_size.length = node.second["length"].as<double>();
        link_size.width = node.second["width"].as<double>();
        link_size.height = node.second["height"].as<double>();
        link_sizes_.push_back(link_size);
    }
}

BoundingBox BoundingBoxes::computeBoundingBox(const geometry_msgs::Pose& pose, const LinkSize& size) {
    const auto& position = pose.position;

    BoundingBox bbox;
    bbox.min_x = position.x - size.length / 2.0;
    bbox.max_x = position.x + size.length / 2.0;
    bbox.min_y = position.y - size.width / 2.0;
    bbox.max_y = position.y + size.width / 2.0;
    bbox.min_z = position.z - size.height / 2.0;
    bbox.max_z = position.z + size.height / 2.0;
    bbox.link_name = size.link_name;

    return bbox;
}

void BoundingBoxes::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    bounding_boxes_.clear();
    rtree_.clear();

    for (const auto& link_size : link_sizes_) {
        auto it = std::find(msg->name.begin(), msg->name.end(), link_size.link_name);
        if (it != msg->name.end()) {
            size_t index = std::distance(msg->name.begin(), it);
            BoundingBox bbox = computeBoundingBox(msg->pose[index], link_size);
            bounding_boxes_[link_size.link_name] = bbox;

            // Insert bounding box into the R-tree
            Box box(Point(bbox.min_x, bbox.min_y, bbox.min_z), Point(bbox.max_x, bbox.max_y, bbox.max_z));
            rtree_.insert(std::make_pair(box, bbox.link_name));
        } else {
            ROS_WARN("Failed to get model state for %s", link_size.link_name.c_str());
        }
    }

    // Publish the bounding boxes after updating
    publishBoundingBoxes();
}

const std::unordered_map<std::string, BoundingBox>& BoundingBoxes::getBoundingBoxes() {
    return bounding_boxes_;
}

void BoundingBoxes::publishBoundingBoxes() {
    quad_msgs::BoundingBoxArray bbox_array;

    for (const auto& pair : bounding_boxes_) {
        const BoundingBox& bbox = pair.second;
        quad_msgs::BoundingBox bbox_msg;
        bbox_msg.min_x = bbox.min_x;
        bbox_msg.max_x = bbox.max_x;
        bbox_msg.min_y = bbox.min_y;
        bbox_msg.max_y = bbox.max_y;
        bbox_msg.min_z = bbox.min_z;
        bbox_msg.max_z = bbox.max_z;
        bbox_msg.link_name = bbox.link_name;

        bbox_array.boxes.push_back(bbox_msg);
    }

    bbox_pub_.publish(bbox_array);
}