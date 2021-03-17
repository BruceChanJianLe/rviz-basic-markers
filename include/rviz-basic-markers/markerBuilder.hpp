#ifndef MARKER_BUILDER_H_
#define MARKER_BUILDER_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


// Forward declaration
class markerAttributes;

class markerBuilder
{
public:
    visualization_msgs::Marker m_;

    visualization_msgs::Marker & marker_;
    // Explicit constructor
    explicit markerBuilder(visualization_msgs::Marker & marker)
    :   marker_(marker)
    {
        ;
    }

    markerBuilder()
    :   marker_(m_)
    {
        ;
    }

    // Conversion operator
    operator visualization_msgs::Marker() const
    {
        return std::move(marker_);
    }

    // Builder facet
    markerAttributes attrb();
};


#endif