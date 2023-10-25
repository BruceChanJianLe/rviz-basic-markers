#ifndef __BSM_H_
#define __BSM_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

#include <string>
#include <vector>

typedef std::array<float, 4> Color;

class marker_node
{
  private:
    // ROS declaration
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Publisher ma_pub_;

    // Variable declaration
    unsigned int rate_;
    visualization_msgs::Marker marker_msg_;
    unsigned int marker_shape_ = visualization_msgs::Marker::CUBE;
    geometry_msgs::Point point_;

    // Private function
    void publish_markers();
    void publish_marker_array();
    unsigned int marker_type_request();

    // Rainbow function from RViz
    static void getRainbowColor(float value, Color& color)
    {
      // this is HSV color palette with hue values going only from 0.0 to 0.833333.

      value = std::min(value, 1.0f);
      value = std::max(value, 0.0f);

      float h = value * 5.0f + 1.0f;
      int i = floor(h);
      float f = h - i;
      if (!(i & 1))
        f = 1 - f; // if i is even
      float n = 1 - f;

      if (i <= 1)
        color[0] = n, color[1] = 0, color[2] = 1;
      else if (i == 2)
        color[0] = 0, color[1] = n, color[2] = 1;
      else if (i == 3)
        color[0] = 0, color[1] = 1, color[2] = n;
      else if (i == 4)
        color[0] = n, color[1] = 1, color[2] = 0;
      else if (i >= 5)
        color[0] = 1, color[1] = n, color[2] = 0;
    }

  public:
    // Constructors and destructor
    marker_node(const std::string &, int);
    marker_node();
    ~marker_node();

    // Expose method
    void start();
};

#endif
