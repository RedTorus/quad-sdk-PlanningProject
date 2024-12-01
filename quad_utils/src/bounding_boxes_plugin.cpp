#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <gazebo_ros/node.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <quad_msgs/BoundingBoxArray.h>
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

typedef bg::model::point<double, 3, bg::cs::cartesian> Point;
typedef bg::model::box<Point> Box;
typedef std::pair<Box, std::string> Value;

struct BoundingBox {
    double min_x;
    double max_x;
    double min_y;
    double max_y;
    double min_z;
    double max_z;
    std::string link_name;
};

class BoundingBoxesPlugin : public gazebo::ModelPlugin {
public:
    BoundingBoxesPlugin() : ModelPlugin() {}

    void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override {
        this->model = _model;
        this->world = _model->GetWorld();
        this->node = gazebo_ros::Node::Get(_sdf);

        // Initialize ROS publisher
        this->bbox_pub = this->node->create_publisher<quad_msgs::BoundingBoxArray>("bounding_boxes", 10);

        // Get link names and sizes from SDF
        if (_sdf->HasElement("link")) {
            sdf::ElementPtr linkElem = _sdf->GetElement("link");
            while (linkElem) {
                LinkSize link_size;
                link_size.link_name = linkElem->Get<std::string>("name");
                link_size.length = linkElem->Get<double>("length");
                link_size.width = linkElem->Get<double>("width");
                link_size.height = linkElem->Get<double>("height");
                this->link_sizes.push_back(link_size);
                linkElem = linkElem->GetNextElement("link");
            }
        }

        // Update bounding boxes periodically
        this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
            std::bind(&BoundingBoxesPlugin::OnUpdate, this));
    }

    void OnUpdate() {
        for (const auto& link_size : this->link_sizes) {
            auto link = this->model->GetLink(link_size.link_name);
            if (link) {
                auto pose = link->WorldPose();
                BoundingBox bbox;
                bbox.min_x = pose.Pos().X() - link_size.length / 2.0;
                bbox.max_x = pose.Pos().X() + link_size.length / 2.0;
                bbox.min_y = pose.Pos().Y() - link_size.width / 2.0;
                bbox.max_y = pose.Pos().Y() + link_size.width / 2.0;
                bbox.min_z = pose.Pos().Z() - link_size.height / 2.0;
                bbox.max_z = pose.Pos().Z() + link_size.height / 2.0;
                bbox.link_name = link_size.link_name;

                // Insert bounding box into the R-tree
                Box box(Point(bbox.min_x, bbox.min_y, bbox.min_z), Point(bbox.max_x, bbox.max_y, bbox.max_z));
                this->rtree.insert(std::make_pair(box, bbox.link_name));

                // Store bounding box
                this->bounding_boxes[link_size.link_name] = bbox;
            }
        }

        // Publish the bounding boxes
        this->publishBoundingBoxes();
    }

    void publishBoundingBoxes() {
        quad_msgs::BoundingBoxArray bbox_array;
        for (const auto& pair : this->bounding_boxes) {
            const BoundingBox& bbox = pair.second;
            quad_msgs::BoundingBox bbox_msg;
            bbox_msg.min_x = bbox.min_x;
            bbox_msg.max_x = bbox.max_x;
            bbox_msg.min_y = bbox.min_y;
            bbox_msg.max_y = bbox.max_y;
            bbox_msg.min_z = bbox.min_z;
            bbox_msg.max_z = bbox.max_z;
            bbox_msg.link_name = bbox.link_name;
            bbox_array.boxes.push_back(bbox_msg);
        }
        this->bbox_pub->publish(bbox_array);
    }

private:
    struct LinkSize {
        std::string link_name;
        double length;
        double width;
        double height;
    };

    gazebo::physics::ModelPtr model;
    gazebo::physics::WorldPtr world;
    gazebo_ros::Node::SharedPtr node;
    rclcpp::Publisher<quad_msgs::BoundingBoxArray>::SharedPtr bbox_pub;
    std::vector<LinkSize> link_sizes;
    std::unordered_map<std::string, BoundingBox> bounding_boxes;
    bgi::rtree<Value, bgi::quadratic<16>> rtree;
    gazebo::event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator