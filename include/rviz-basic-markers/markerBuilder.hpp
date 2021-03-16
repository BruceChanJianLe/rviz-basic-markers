#ifndef MARKER_BUILDER_H_
#define MARKER_BUILDER_H_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>


// Forward declaration
class markerAttributes;

class markerBuilder
{
private:
    visualization_msgs::Marker m_;

protected:
    visualization_msgs::Marker & marker_;
    // Explicit constructor
    explicit markerBuilder(visualization_msgs::Marker & marker)
    :   marker_(marker)
    {
        ;
    }

public:
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
    inline markerAttributes attrb();

    friend class markerAttributes;
};


#endif