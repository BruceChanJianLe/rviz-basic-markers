#ifndef __BSM_H_
#define __BSM_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <string>

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

        // Private function
        void publish_markers();

    public:
        // Constructors and destructor
        marker_node(const std::string &, int);
        marker_node();
        ~marker_node();

        // Expose method
        void start();
};

#endif