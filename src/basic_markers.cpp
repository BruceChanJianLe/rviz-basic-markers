#include "rviz-basic-markers/basic_markers.hpp"
#include "visualization_msgs/MarkerArray.h"


marker_node::marker_node(const std::string & pub_name, int rate)
{
  // Set publisher topic
  pub_ = nh_.advertise<visualization_msgs::Marker>(pub_name, 10);
  ma_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(pub_name + "_rainbow", 10);
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

    publish_marker_array();
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

  // Set marker shape
  marker_msg_.type = marker_type_request();

  // Publish marker msg
  pub_.publish(marker_msg_);
}


unsigned int marker_node::marker_type_request()
{
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
    case visualization_msgs::Marker::SPHERE_LIST:
      {
        // Set marker scale
        marker_msg_.scale.x = 0.1;      // Point width
        marker_msg_.scale.y = 0.1;      // Point height
        marker_msg_.scale.z = 0.0;      // Not used
                                        // Set each position for each point
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
        marker_shape_ = visualization_msgs::Marker::POINTS;
        break;
      }
    case visualization_msgs::Marker::POINTS:
      {
        // Set marker scale
        marker_msg_.scale.x = 0.0;      // Not used
        marker_msg_.scale.y = 0.0;      // Not used
        marker_msg_.scale.z = 0.5;      // Specifies the height of an uppercase "A"
        marker_msg_.text = "This is a demo!";
        marker_shape_ = visualization_msgs::Marker::TEXT_VIEW_FACING;
        break;
      }
    case visualization_msgs::Marker::TEXT_VIEW_FACING:
      {
        // Set marker scale
        marker_msg_.scale.x = 1.0;
        marker_msg_.scale.y = 1.0;
        marker_msg_.scale.z = 1.0;
        // Provide path to mesh
        marker_msg_.mesh_resource = "package://rviz-basic-markers/meshes/elegant_male.dae";
        // Set meshes color to true
        marker_msg_.mesh_use_embedded_materials = true;
        // Unset given color
        marker_msg_.color.r = 0;
        marker_msg_.color.g = 0;
        marker_msg_.color.b = 0;
        marker_msg_.color.a = 0;
        marker_shape_ = visualization_msgs::Marker::MESH_RESOURCE;
        break;
      }
    case visualization_msgs::Marker::MESH_RESOURCE:
      {
        // Set marker scale
        marker_msg_.scale.x = 1.0;
        marker_msg_.scale.y = 1.0;
        marker_msg_.scale.z = 1.0;
        // Set the lines  0-1-2, 3-4-5, ...
        point_.x = 0.0, point_.y = 0.0, point_.z = 0.0;
        marker_msg_.points.emplace_back(point_);
        point_.x = 1.0, point_.y = 0.0, point_.z = 0.0;
        marker_msg_.points.emplace_back(point_);
        point_.x = 1.0, point_.y = 1.0, point_.z = 0.0;
        marker_msg_.points.emplace_back(point_);
        marker_shape_ = visualization_msgs::Marker::TRIANGLE_LIST;
        break;
      }
    case visualization_msgs::Marker::TRIANGLE_LIST:
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

  return marker_shape_;
}

void marker_node::publish_marker_array()
{
  // Create 10 markers
  visualization_msgs::MarkerArray ma_msg;
  // Give them color
  size_t s_size = 20;
  for (size_t i = 0; i < s_size; ++i)
  {
    visualization_msgs::Marker m_msg;
    m_msg.header.frame_id = "map";
    m_msg.header.stamp = ros::Time::now();
    // Any marker sent with the same namespace and id will overwrite the old one
    m_msg.ns = "rainbow";
    m_msg.id = i;
    // Set marker action {ADD, DELETE, DELETEALL}
    m_msg.action = visualization_msgs::Marker::ADD;
    // Set the pose of marker
    m_msg.pose.position.x = i;
    m_msg.pose.position.y = i;
    m_msg.pose.position.z = 0;
    m_msg.pose.orientation.x = 0.0;
    m_msg.pose.orientation.y = 0.0;
    m_msg.pose.orientation.z = 0.0;
    m_msg.pose.orientation.w = 1.0;
    // Set marker color
    Color color {0, 0, 0, 0};
    getRainbowColor(i / static_cast<float>(s_size), color);
    m_msg.color.r = color[0];
    m_msg.color.g = color[1];
    m_msg.color.b = color[2];
    m_msg.color.a = 1.0;
    // Set marker reset time, timer reset when new one is received
    m_msg.lifetime = ros::Duration();

    // Set marker shape
    m_msg.type = visualization_msgs::Marker::SPHERE;

    m_msg.scale.x = 1.0;
    m_msg.scale.y = 1.0;
    m_msg.scale.z = 1.0;

    ma_msg.markers.push_back(m_msg);
  }
  // Publish
  ma_pub_.publish(ma_msg);
}
