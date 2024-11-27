#include <quad_utils/collision_checker.h>
#include <ros/ros.h> // For logging

CollisionChecker::CollisionChecker(const BoundingBoxes& bounding_boxes)
    : bounding_boxes_(bounding_boxes) {}

bool CollisionChecker::isInCollision(const geometry_msgs::Point& point) const {
    // Get the latest bounding boxes
    const auto& boxes = bounding_boxes_.getBoundingBoxes();
    //ROS_INFO("CCHECKER point: [%f, %f, %f]", point.x, point.y, point.z);
    //ROS_INFO("CCHECKER boxes size: %d", boxes.size());
    // Check if the point is within any bounding box
    for (const auto& [link_name, box] : boxes) {
        //ROS_INFO("box.min_x: %f box.max_x: %f box.min_y: %f box.max_y: %f", box.min_x, box.max_x, box.min_y, box.max_y);	
        if (point.x >= box.min_x && point.x <= box.max_x && point.y >= box.min_y && point.y <= box.max_y && point.z >= box.min_z && point.z <= box.max_z) {
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

double CollisionChecker::isInCollisionZ(double z, const double& h_min, const double& h_max) const {
    // Get the latest bounding boxes
    const auto& boxes = bounding_boxes_.getBoundingBoxes();
    //ROS_INFO("CCHECKER point: [%f, %f, %f]", point.x, point.y, point.z);
    //ROS_INFO("CCHECKER boxes size: %d", boxes.size());
    // Check if the point is within any bounding box
    bool collided = false;

    for (const auto& [link_name, box] : boxes) {
        if (link_name == "table_top"){
            if (z >= box.min_z && z <= box.max_z) {
                std::cout << "h_nom failed" << std::endl;
                collided = true; // Point is inside a bounding box
            }

            if (collided){
                std::cout << "Checking h_min" << std::endl;
                if (h_min < box.min_z){
                    std::cout << "h_min passed: " << h_min << " box: " << box.min_z << std::endl;
                    z = 0.075;
                    collided = false;
                    break;
                }
            }
            
            if (collided){
                std::cout << "Checking h_max" << std::endl;
                if (h_max > box.max_z){
                    std::cout << "h_max passed: " << h_max << " box: " << box.max_z << std::endl;
                    z = 0.375 ;
                    collided = false;
                    break;
                }
            }
        }
    }

    if (collided){
        ROS_WARN_STREAM("Z Collision detected with link:");
        return 0;
    }
    else{
        return z;
    }
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
