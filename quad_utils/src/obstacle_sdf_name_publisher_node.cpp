#include "ros/ros.h"
#include "quad_utils/obstacle_sdf_name_publisher.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_sdf_name_publisher");
    ros::NodeHandle nh;

    BoxSdfPublisher publisher(nh);
    publisher.run();

    return 0;
}
