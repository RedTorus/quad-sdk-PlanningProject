#include "quad_utils/bounding_boxes.h"
#include <yaml-cpp/yaml.h>

BoundingBoxes::BoundingBoxes(ros::NodeHandle& nh, const std::string& yaml_file)
    : nh_(nh) {
    link_state_client_ = nh_.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    loadLinkSizes(yaml_file);
    updateBoundingBoxes();
}

void BoundingBoxes::updateBoundingBoxes() {
    for (const auto& link_size : link_sizes_) {
        gazebo_msgs::GetLinkState link_state_srv;
        link_state_srv.request.link_name = link_size.link_name;
        link_state_srv.request.reference_frame = "world";

        if (link_state_client_.call(link_state_srv)) {
            if (link_state_srv.response.success) {
                BoundingBox bbox = computeBoundingBox(link_state_srv.response.link_state, link_size);
                bounding_boxes_[link_size.link_name] = bbox;
            } else {
                ROS_WARN("Failed to get link state for %s", link_size.link_name.c_str());
            }
        } else {
            ROS_ERROR("Failed to call service /gazebo/get_link_state for %s", link_size.link_name.c_str());
        }
    }
}

BoundingBox BoundingBoxes::computeBoundingBox(const gazebo_msgs::LinkState& link_state, const LinkSize& size) {
    const auto& position = link_state.pose.position;

    BoundingBox bbox;
    bbox.min_x = position.x - size.length / 2.0;
    bbox.max_x = position.x + size.length / 2.0;
    bbox.min_y = position.y - size.width / 2.0;
    bbox.max_y = position.y + size.width / 2.0;
    bbox.min_z = position.z - size.height / 2.0;
    bbox.max_z = position.z + size.height / 2.0;

    // std::cout << "BB position.z: " << position.z << " BB size.height: " << size.height << std::endl;
    // std::cout << "BB minz: " << bbox.min_z << " BB maxz: " << bbox.max_z << std::endl;

    return bbox;
}

const std::unordered_map<std::string, BoundingBox>& BoundingBoxes::getBoundingBoxes() const {
    return bounding_boxes_;
}

void BoundingBoxes::loadLinkSizes(const std::string& yaml_file) {
    YAML::Node config = YAML::LoadFile(yaml_file); // Load the YAML file
    for (const auto& node : config) {
        LinkSize link_size;
        link_size.link_name = node.first.as<std::string>();
        link_size.length = node.second["length"].as<double>();
        link_size.width = node.second["width"].as<double>();
        link_size.height = node.second["height"].as<double>();
        link_sizes_.push_back(link_size);
    }
}
