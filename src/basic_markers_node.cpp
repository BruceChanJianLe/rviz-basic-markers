#include "rviz-basic-markers/basic_markers.hpp"


const std::string RosNodeName = "basic_markers_node";
const std::string pub_name = "basic_markers_topic";


int main(int argc, char ** argv)
{
    ros::init(argc, argv, RosNodeName);

    marker_node node(pub_name, 1);

    node.start();

    return 0;
}