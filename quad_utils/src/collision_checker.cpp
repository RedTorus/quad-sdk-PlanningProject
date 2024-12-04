#include <quad_utils/collision_checker.h>
#include <ros/ros.h> // For logging

CollisionChecker::CollisionChecker(ros::NodeHandle& nh) {
    bbox_sub_ = nh.subscribe("bounding_boxes", 10, &CollisionChecker::boundingBoxesCallback, this);
}

void CollisionChecker::boundingBoxesCallback(const quad_msgs::BoundingBoxArray::ConstPtr& msg) {
    bounding_boxes_.clear();
    for (const auto& bbox_msg : msg->boxes) {
        BoundingBox bbox;
        bbox.min_x = bbox_msg.min_x;
        bbox.max_x = bbox_msg.max_x;
        bbox.min_y = bbox_msg.min_y;
        bbox.max_y = bbox_msg.max_y;
        bbox.min_z = bbox_msg.min_z;
        bbox.max_z = bbox_msg.max_z;
        bbox.link_name = bbox_msg.link_name;
        bounding_boxes_[bbox.link_name] = bbox;
    }
}

bool CollisionChecker::isInCollision(const geometry_msgs::Point& point) const {
    // Check if the point is within any bounding box 

    std::map<std::string, bool> collision_map;

    for (const auto& [link_name, box] : bounding_boxes_) {
        // std::cout << "Checking for collision with link: " << link_name << std::endl;
        // std::cout << "Point: " << point.x << ", " << point.y << ", " << point.z << std::endl;
        // std::cout << "Bounding box: " << box.min_x << ", " << box.max_x << ", " << box.min_y << ", " << box.max_y << ", " << box.min_z << ", " << box.max_z << std::endl;
        bool x = point.x >= box.min_x - 0.01 && point.x <= box.max_x + 0.01;
        bool y = point.y >= box.min_y - 0.01 && point.y <= box.max_y + 0.01;
        bool z = point.z >= box.min_z - 0.01 && point.z <= box.max_z + 0.01;
        // std::cout << "x: " << x << std::endl;
        // std::cout << "y: " << y << std::endl;
        // std::cout << "z: " << z << std::endl;
        if (x && y) {
            collision_map[link_name] = true;
        } 
    }

    for (const auto& [link_name, collision] : collision_map) {
        if (collision) {
            ROS_WARN("Collision detected with link: %s", link_name.c_str());
            return true;
        }
    }
    return false;
}