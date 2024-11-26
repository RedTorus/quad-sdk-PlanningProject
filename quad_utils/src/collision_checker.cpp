#include <quad_utils/collision_checker.h>
#include <ros/ros.h> // For logging

CollisionChecker::CollisionChecker(const BoundingBoxes& bounding_boxes)
    : bounding_boxes_(bounding_boxes) {}

bool CollisionChecker::isInCollision(const geometry_msgs::Point& point) const {
    // Get the latest bounding boxes
    const auto& boxes = bounding_boxes_.getBoundingBoxes();

    // Check if the point is within any bounding box
    for (const auto& [link_name, box] : boxes) {
        if (
            point.x >= box.min_x - 0.8 && point.x <= box.max_x + 0.8 ) {
            ROS_WARN_STREAM("Collision detected with link: " << link_name);
            return true; // Point is inside a bounding box
        }
        /* if (point.x >= box.min_x - 1.3 && point.x <= box.max_x + 1.3 &&
            point.y >= box.min_y - 1.3 && point.y <= box.max_y + 1.3 &&
            point.z >= box.min_z - 0.3 && point.z <= box.max_z) {
            ROS_WARN_STREAM("Collision detected with link: " << link_name);
            return true; // Point is inside a bounding box
        } */
    }

    // No collision detected
    return false;
}

bool CollisionChecker::isInTransformedCollision(const geometry_msgs::Point& point,const Eigen::Matrix3d& R_mat) const {
    // Get the latest bounding boxes
    const auto& boxes = bounding_boxes_.getBoundingBoxes();

    // Check if the point is within any bounding box
    for (const auto& [link_name, box] : boxes) {
        Eigen::Vector3d point_eigen;
        point_eigen << point.x, point.y, point.z;
        Eigen::Vector3d transformed_point = R_mat * point_eigen;
        // do both rotatoin and translation
 
        if (transformed_point.x() >= box.min_x && transformed_point.x() <= box.max_x &&
            transformed_point.y() >= box.min_y && transformed_point.y() <= box.max_y &&
            transformed_point.z() >= box.min_z && transformed_point.z() <= box.max_z) {
            ROS_WARN_STREAM("Collision detected with link: " << link_name);
            return true; // Point is inside a bounding box
        }
    }

    // No collision detected
    return false;
}

bool CollisionChecker::isInTransformedCollision2(const geometry_msgs::Point& point,const Eigen::Matrix3d& R_mat, const Eigen::Vector3d& T) const {
    // Get the latest bounding boxes
    const auto& boxes = bounding_boxes_.getBoundingBoxes();

    // Check if the point is within any bounding box
    for (const auto& [link_name, box] : boxes) {
        Eigen::Vector3d point_eigen;
        point_eigen << point.x, point.y, point.z;
        Eigen::Vector3d transformed_point = R_mat * point_eigen + T;
        // do both rotatoin and translation
 
        if (transformed_point.x() >= box.min_x && transformed_point.x() <= box.max_x &&
            transformed_point.y() >= box.min_y && transformed_point.y() <= box.max_y &&
            transformed_point.z() >= box.min_z && transformed_point.z() <= box.max_z) {
            ROS_WARN_STREAM("Collision detected with link: " << link_name);
            return true; // Point is inside a bounding box
        }
    }

    // No collision detected
    return false;
}
