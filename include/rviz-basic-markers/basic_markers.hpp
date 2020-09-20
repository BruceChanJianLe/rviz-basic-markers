#ifndef __BSM_H_
#define __BSM_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

#include <string>
#include <vector>

class marker_node
{
    private:
        // ROS declaration
        ros::NodeHandle nh_;
        ros::Publisher pub_;

        // Variable declaration
        unsigned int rate_;
        visualization_msgs::Marker marker_msg_;
        unsigned int marker_shape_ = visualization_msgs::Marker::CUBE;
        geometry_msgs::Point point_;

        // Private function
        void publish_markers();
        unsigned int marker_type_request();

    public:
        // Constructors and destructor
        marker_node(const std::string &, int);
        marker_node();
        ~marker_node();

        // Expose method
        void start();
};

#endif