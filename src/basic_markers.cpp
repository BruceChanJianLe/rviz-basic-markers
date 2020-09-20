#include "rviz-basic-markers/basic_markers.hpp"


marker_node::marker_node(const std::string & pub_name, int rate)
{
    // Set publisher topic
    pub_ = nh_.advertise<visualization_msgs::Marker>(pub_name, 10);
    // Set ROS node rate
    rate_ = rate;
}


marker_node::marker_node()
{
    ;
}


marker_node::~marker_node()
{
    ;
}


void marker_node::start()
{
    // Set ROS node sleep rate
    ros::Rate r(rate_);

    // Perform
    while(ros::ok())
    {
        // Publish markers
        publish_markers();

        // Sleep
        r.sleep();
    }
}


void marker_node::publish_markers()
{
    // Clear up marker msg
    marker_msg_.points.clear();     // Particulary for points and scale as usages differ from markers to markers
    // Set relative frame and time
    marker_msg_.header.frame_id = "map";
    marker_msg_.header.stamp = ros::Time::now();
    // Any marker sent with the same namespace and id will overwrite the old one
    marker_msg_.ns = "basic_shape";
    marker_msg_.id = 0;
    // Set marker action {ADD, DELETE, DELETEALL}
    marker_msg_.action = visualization_msgs::Marker::ADD;
    // Set the pose of marker
    marker_msg_.pose.position.x = 0;
    marker_msg_.pose.position.y = 0;
    marker_msg_.pose.position.z = 0;
    marker_msg_.pose.orientation.x = 0.0;
    marker_msg_.pose.orientation.y = 0.0;
    marker_msg_.pose.orientation.z = 0.0;
    marker_msg_.pose.orientation.w = 1.0;
    // Set marker color
    marker_msg_.color.r = 0.0f;
    marker_msg_.color.g = 1.0f;
    marker_msg_.color.b = 0.0f;
    marker_msg_.color.a = 1.0;
    // Set marker reset time, timer reset when new one is received
    marker_msg_.lifetime = ros::Duration();

    // Cycle between different shapes
    switch (marker_shape_)
    {
        case visualization_msgs::Marker::ARROW:
        {
            // Set marker scale -- 1x1x1 here means 1m on each side
            marker_msg_.scale.x = 1.0;
            marker_msg_.scale.y = 1.0;
            marker_msg_.scale.z = 1.0;
            marker_shape_ = visualization_msgs::Marker::CUBE;
            break;
        }
        case visualization_msgs::Marker::CUBE:
            // Set marker scale -- 1x1x1 here means 1m on each side
            marker_msg_.scale.x = 1.0;
            marker_msg_.scale.y = 1.0;
            marker_msg_.scale.z = 1.0;
            marker_shape_ = visualization_msgs::Marker::SPHERE;
            break;
        case visualization_msgs::Marker::SPHERE:
        {
            // Set marker scale
            marker_msg_.scale.x = 1.0;      // Diameter in x direction
            marker_msg_.scale.y = 1.0;      // Diameter in y direction
            marker_msg_.scale.z = 1.0;      // Cylinder height
            marker_shape_ = visualization_msgs::Marker::CYLINDER;
            break;
        }
        case visualization_msgs::Marker::CYLINDER:
        {
            // Set marker scale
            marker_msg_.scale.x = 0.1;      // Width of line segments
            marker_msg_.scale.y = 0.0;      // Not used
            marker_msg_.scale.z = 0.0;      // Not used
            // Set the lines 0-1, 1-2, 2-3, ...
            point_.x = 0.0, point_.y = -2.5, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.5, point_.y = -1.5, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = -0.5, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.5, point_.y = 0.5, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = 1.5, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.5, point_.y = 2.5, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            marker_shape_ = visualization_msgs::Marker::LINE_STRIP;
            break;
        }
        case visualization_msgs::Marker::LINE_STRIP:
        {
            // Set marker scale
            marker_msg_.scale.x = 0.1;      // Width of line segments
            marker_msg_.scale.y = 0.0;      // Not used
            marker_msg_.scale.z = 0.0;      // Not used
            // Set the lines 0-1, 2-3, 4-5, ...
            point_.x = 0.0, point_.y = -2.5, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = -2.5, point_.z = 1.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = -1.5, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = -1.5, point_.z = -1.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = -0.5, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = -0.5, point_.z = 1.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = 0.5, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = 0.5, point_.z = -1.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = 1.5, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = 1.5, point_.z = 1.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = 2.5, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = 2.5, point_.z = -1.0;
            marker_msg_.points.emplace_back(point_);
            marker_shape_ = visualization_msgs::Marker::LINE_LIST;
            break;
        }
        case visualization_msgs::Marker::LINE_LIST:
        {
            // Set marker scale
            marker_msg_.scale.x = 0.1;
            marker_msg_.scale.y = 0.1;
            marker_msg_.scale.z = 0.1;
            // Set each position for each cube
            point_.x = 0.0, point_.y = -2.5, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = -2.5, point_.z = 1.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = -1.5, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = -1.5, point_.z = -1.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = -0.5, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = -0.5, point_.z = 1.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = 0.5, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = 0.5, point_.z = -1.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = 1.5, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = 1.5, point_.z = 1.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = 2.5, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = 2.5, point_.z = -1.0;
            marker_msg_.points.emplace_back(point_);
            marker_shape_ = visualization_msgs::Marker::CUBE_LIST;
            break;
        }
        case visualization_msgs::Marker::CUBE_LIST:
        {
            // Set marker scale
            marker_msg_.scale.x = 0.2;
            marker_msg_.scale.y = 0.2;
            marker_msg_.scale.z = 0.2;
            // Set each position for each sphere
            point_.x = 0.0, point_.y = 0.0, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 1.0, point_.y = 0.0, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 1.0, point_.y = 1.0, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = 1.0, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 1.0, point_.y = -1.0, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = -1.0, point_.y = 0.0, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = -1.0, point_.y = -1.0, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = 0.0, point_.y = -1.0, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            point_.x = -1.0, point_.y = 1.0, point_.z = 0.0;
            marker_msg_.points.emplace_back(point_);
            marker_shape_ = visualization_msgs::Marker::SPHERE_LIST;
            break;
        }
        // case visualization_msgs::Marker::POINTS:
        // {
        //     // Set marker scale -- 1x1x1 here means 1m on each side
        //     marker_msg_.scale.x = 1.0;
        //     marker_msg_.scale.y = 1.0;
        //     marker_msg_.scale.z = 1.0;
        //     marker_shape_ = visualization_msgs::Marker::CUBE;
        //     break;
        // }
        default:
        {
            // Method 1 of defining an arrow
            // Set marker scale
            marker_msg_.scale.x = 0.1;      // Shaft diameter
            marker_msg_.scale.y = 0.2;      // Head diameter
            marker_msg_.scale.z = 0.1;      // Head length
            // Set start and end point at(0) and at(1)
            point_.x = 0.0, point_.y = 0.0, point_.z = 0.0;
            marker_msg_.points.push_back(point_);
            point_.x = 1.0, point_.y = 1.0, point_.z = 0.0;
            marker_msg_.points.push_back(point_);
            marker_shape_ = visualization_msgs::Marker::ARROW;

            // Method 2 of defining an arrow
            // Set marker scale
            // marker_msg_.scale.x = 1.0;      // Arrow length
            // marker_msg_.scale.y = 0.1;      // Arrow width
            // marker_msg_.scale.z = 0.1;      // Arrow height
            // marker_shape_ = visualization_msgs::Marker::ARROW;
        }
    }

    // Set marker shape
    marker_msg_.type = marker_shape_;

    // Publish marker msg
    pub_.publish(marker_msg_);
}