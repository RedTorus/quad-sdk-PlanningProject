#include <quad_utils/collision_checker.h>
#include <ros/ros.h> // For logging


CollisionChecker::CollisionChecker(const BoundingBoxes& bounding_boxes)
    : bounding_boxes_(bounding_boxes) {}

bool CollisionChecker::isInCollision(const geometry_msgs::Point& point) const {
    // Get the latest bounding boxes
    //bounding_boxes_.updateBoundingBoxes();
    const auto& boxes = bounding_boxes_.getBoundingBoxes();
    //ROS_INFO("CCHECKER point: [%f, %f, %f]", point.x, point.y, point.z);

    //ROS_INFO("CCHECKER boxes size: %d", boxes.size());
    // Check if the point is within any bounding box
    for (const auto& [link_name, box] : boxes) {
        //ROS_INFO("box.min_x: %f box.max_x: %f box.min_y: %f box.max_y: %f", box.min_x, box.max_x, box.min_y, box.max_y);	
        if (point.x >= box.min_x -0.05 && point.x <= box.max_x + 0.05 && point.y >= box.min_y - 0.05 && point.y <= box.max_y + 0.05){ //&& point.z >= box.min_z && point.z <= box.max_z) {
            //ROS_WARN_STREAM("Collision detected with link: " << link_name);
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

bool CollisionChecker::isInExpandedCollision(const geometry_msgs::Point& point, const Eigen::Vector3d& velocity, double dt) {
    // Get the latest bounding boxes
    //bounding_boxes_.updateBoundingBoxes();
    const auto& boxes = bounding_boxes_.getBoundingBoxes();
    
    // Check if the point is within any expanded bounding box
    for (const auto& [link_name, box] : boxes) {
        BoundingBox expanded_box = bounding_boxes_.computeSlidingWindowBoundingBox(box, velocity, dt);

        if (point.x >= expanded_box.min_x -0.05 && point.x <= expanded_box.max_x + 0.05 &&
            point.y >= expanded_box.min_y -0.05 && point.y <= expanded_box.max_y +0.05) {//&&
            //point.z >= expanded_box.min_z && point.z <= expanded_box.max_z) {
            return true; // Point is inside an expanded bounding box
        }
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
