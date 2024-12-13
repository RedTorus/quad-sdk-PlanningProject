#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H

#include <quad_utils/bounding_boxes.h>
#include <geometry_msgs/Point.h>
#include <unordered_map>
#include <string>
#include <Eigen/Dense>

class CollisionChecker {
public:
    /**
     * Constructor
     * @param bounding_boxes Reference to the BoundingBoxes object.
     */
    CollisionChecker() = default;
    CollisionChecker(const BoundingBoxes& bounding_boxes);

    /**
     * Check if a given point is within any bounding box.
     * @param point The point to check (robot's state position).
     * @return True if the point is within any bounding box, otherwise false.
     */
    bool isInCollision(const geometry_msgs::Point& point) const;

    bool isInExpandedCollision(const geometry_msgs::Point& point, const Eigen::Vector3d& velocity, double dt);

    bool isInTransformedCollision(const geometry_msgs::Point& point,const Eigen::Matrix3d& R_mat) const;

    bool isInTransformedCollision2(const geometry_msgs::Point& point,const Eigen::Matrix3d& R_mat, const Eigen::Vector3d& T) const;

private:
    const BoundingBoxes& bounding_boxes_; // Reference to the bounding boxes object
};

#endif // COLLISION_CHECKER_H
