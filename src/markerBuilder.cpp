#include "rviz-basic-markers/markerBuilder.hpp"
#include "rviz-basic-markers/markerAttributes.hpp"


markerAttributes markerBuilder::attrb()
{
    return markerAttributes(marker_);
}