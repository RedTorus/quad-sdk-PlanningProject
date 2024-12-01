#ifndef BOUNDING_BOXES_H
#define BOUNDING_BOXES_H

#include <ros/ros.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/ModelStates.h>
#include <string>
#include <unordered_map>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <quad_msgs/BoundingBoxArray.h> // Include the custom message header

// Define a structure for link sizes
struct LinkSize {
    std::string link_name;
    double length;
    double width;
    double height;
};

// Define a structure for bounding boxes
struct BoundingBox {
    double min_x;
    double max_x;
    double min_y;
    double max_y;
    double min_z;
    double max_z;
    std::string link_name;
};

// Define types for Boost.Geometry
namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<double, 3, bg::cs::cartesian> Point;
typedef bg::model::box<Point> Box;
typedef std::pair<Box, std::string> Value;

// Class for managing bounding boxes
class BoundingBoxes {
public:
    BoundingBoxes(ros::NodeHandle& nh, const std::string& yaml_file);

    const std::unordered_map<std::string, BoundingBox>& getBoundingBoxes() const;
    void publishBoundingBoxes(); // Add method to publish bounding boxes

private:
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    BoundingBox computeBoundingBox(const geometry_msgs::Pose& pose, const LinkSize& size);
    void loadLinkSizes(const std::string& yaml_file);

    ros::NodeHandle nh_;
    ros::Subscriber model_states_sub_;
    ros::Publisher bbox_pub_; // ROS publisher for bounding boxes
    std::vector<LinkSize> link_sizes_;
    std::unordered_map<std::string, BoundingBox> bounding_boxes_;
    bgi::rtree<Value, bgi::quadratic<16>> rtree_; // R-tree for bounding boxes
};

#endif // BOUNDING_BOXES_H