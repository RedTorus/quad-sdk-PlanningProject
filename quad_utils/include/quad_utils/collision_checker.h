#ifndef COLLISION_CHECKER_H
#define COLLISION_CHECKER_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <quad_msgs/BoundingBoxArray.h>
#include <unordered_map>
#include <string>
#include "quad_utils/bounding_boxes.h"

class CollisionChecker {
public:
    CollisionChecker(ros::NodeHandle& nh);

    bool isInCollision(const geometry_msgs::Point& point) const;

private:
    void boundingBoxesCallback(const quad_msgs::BoundingBoxArray::ConstPtr& msg);

    ros::Subscriber bbox_sub_;
    std::unordered_map<std::string, BoundingBox> bounding_boxes_;
};

#endif // COLLISION_CHECKER_H