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
        // std::cout << "Link name: " << link_name << std::endl;
        // std::cout << "boxmax.z: " << box.max_z << " boxmin z: " << box.min_z << " point.x: " << point.x << " box.min_x: " << box.min_x
        // << " box.max_x: " << box.max_x << " point.y: " << point.y << " box.min_y: " << box.min_y << " box.max_y: " << box.max_y << std::endl;
        if (box.min_z-0.03 < 0.13 && point.x >= box.min_x-0.03 && point.x <= box.max_x+0.03 && point.y >= box.min_y-0.03 && point.y <= box.max_y+0.03) {
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
    bool collided = false;

    for (const auto& [link_name, box] : boxes) {
        // std::cout << "box min: " << box.min_z << std::endl; 
        if (z >= box.min_z-0.1 && z <= box.max_z+0.1) {
            // std::cout << "h_nom collided" << std::endl;
            collided = true; // Point is inside a bounding box
        }

        if (collided){
            // std::cout << "Checking h_min: " << h_min << " boxmin: " << box.min_z << std::endl;
            if (h_min < box.min_z-0.1){
                // std::cout << "h_min passed: " << h_min << " box: " << box.min_z << std::endl;
                z = 0.1;
                collided = false;
                break;
            }
        } 

        // if (collided){
        //     std::cout << "Checking h_max" << std::endl;
        //     if (h_max > box.max_z+0.03){
        //         std::cout << "h_max passed: " << h_max << " box: " << box.max_z << std::endl;
        //         z = 0.375;
        //         collided = false;
        //         break;
        //     }
        // }
    }

    if (collided){
        // ROS_WARN_STREAM("Z Collision detected with link:");
        return 0;
    }
    else{
        // std::cout << "Collision checking done" << std::endl;
        return z;
    }
}

bool CollisionChecker::isInExpandedCollision(const geometry_msgs::Point& point, const Eigen::Vector3d& velocity, double dt) const {
    // Get the latest bounding boxes
    const auto& boxes = bounding_boxes_.getBoundingBoxes();

    // Check if the point is within any expanded bounding box
    for (const auto& [link_name, box] : boxes) {
        BoundingBox expanded_box = bounding_boxes_.computeSlidingWindowBoundingBox(box, velocity, dt);

        if (point.x >= expanded_box.min_x && point.x <= expanded_box.max_x &&
            point.y >= expanded_box.min_y && point.y <= expanded_box.max_y &&
            point.z >= expanded_box.min_z && point.z <= expanded_box.max_z) {
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
