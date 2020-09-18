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
    // Set relative frame and time
    marker_msg_.header.frame_id = "map";
    marker_msg_.header.stamp = ros::Time::now();
    // Any marker sent with the same namespace and id will overwrite the old one
    marker_msg_.ns = "basic_shape";
    marker_msg_.id = 0;
    // Set marker shape
    marker_msg_.type = marker_shape_;
    // Set marker action {ADD, DELETE, DELETEALL}
    marker_msg_.action = visualization_msgs::Marker::ADD;
    // Set the pose of marker
    marker_msg_.pose.position.x = 0;
    marker_msg_.pose.position.y = 0;
    marker_msg_.pose.position.z = 1.0;
    marker_msg_.pose.orientation.x = 0.0;
    marker_msg_.pose.orientation.y = 0.0;
    marker_msg_.pose.orientation.z = 0.0;
    marker_msg_.pose.orientation.w = 1.0;
    // Set marker scale -- 1x1x1 here means 1m on each side
    marker_msg_.scale.x = 1.0;
    marker_msg_.scale.y = 1.0;
    marker_msg_.scale.z = 1.0;
    // Set marker color
    marker_msg_.color.r = 0.0f;
    marker_msg_.color.g = 1.0f;
    marker_msg_.color.b = 0.0f;
    marker_msg_.color.a = 1.0;
    // Set marker reset time, timer reset when new one is received
    marker_msg_.lifetime = ros::Duration();

    // Publish marker msg
    pub_.publish(marker_msg_);

    // Cycle between different shapes
    switch (marker_shape_)
    {
    case visualization_msgs::Marker::CUBE:
      marker_shape_ = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      marker_shape_ = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      marker_shape_ = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      marker_shape_ = visualization_msgs::Marker::CUBE;
      break;
    }
}