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
    for (const auto& [link_name, box] : bounding_boxes_) {
        if (point.x >= box.min_x - 0.01 && point.x <= box.max_x + 0.01 &&
            point.y >= box.min_y - 0.01 && point.y <= box.max_y + 0.01 &&
            point.z >= box.min_z - 0.01 && point.z <= box.max_z + 0.01) {
            ROS_WARN_STREAM("Collision detected with link: " << link_name);
            return true; // Point is inside a bounding box
        }
    }
    return false; // No collision detected
}