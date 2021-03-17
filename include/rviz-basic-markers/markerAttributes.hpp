#pragma once

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>



class markerAttributes : public markerBuilder
{
    private:
        typedef markerAttributes Self;

    public:
        explicit markerAttributes(visualization_msgs::Marker & marker)
        :   markerBuilder(marker)
        {
            ;
        }

        // Fuild Interface
        Self & frame_id(std::string frame_id)
        {
            marker_.header.frame_id = frame_id;
            return *this;
        }

        Self & stamp(ros::Time stamp)
        {
            marker_.header.stamp = stamp;
            return *this;
        }

        Self & ns(std::string ns)
        {
            marker_.ns = ns;
            return *this;
        }

        Self & id(int id)
        {
            marker_.id = id;
            return *this;
        }

        Self & action(int32_t action)
        {
            marker_.action = action;
            return *this;
        }

        Self & position_x(double position_x)
        {
            marker_.pose.position.x = position_x;
            return *this;
        }

        Self & position_y(double position_y)
        {
            marker_.pose.position.y = position_y;
            return *this;
        }

        Self & position_z(double position_z)
        {
            marker_.pose.position.z = position_z;
            return *this;
        }

        Self & orientation_x(double orientation_x)
        {
            marker_.pose.orientation.x = orientation_x;
            return *this;
        }

        Self & orientation_y(double orientation_y)
        {
            marker_.pose.orientation.y = orientation_y;
            return *this;
        }

        Self & orientation_z(double orientation_z)
        {
            marker_.pose.orientation.z = orientation_z;
            return *this;
        }

        Self & orientation_w(double orientation_w)
        {
            marker_.pose.orientation.w = orientation_w;
            return *this;
        }

        Self & color_r(float color_r)
        {
            marker_.color.r = color_r;
            return *this;
        }

        Self & color_g(float color_g)
        {
            marker_.color.g = color_g;
            return *this;
        }

        Self & color_b(float color_b)
        {
            marker_.color.b = color_b;
            return *this;
        }

        Self & color_a(float color_a)
        {
            marker_.color.a = color_a;
            return *this;
        }

        Self & lifetime(ros::Duration duration)
        {
            marker_.lifetime = duration;
            return *this;
        }

        Self & type(int32_t type)
        {
            marker_.type = type;
            return *this;
        }

        Self & scale_x(double scale_x)
        {
            marker_.scale.x = scale_x;
            return *this;
        }

        Self & scale_y(double scale_y)
        {
            marker_.scale.y = scale_y;
            return *this;
        }

        Self & scale_z(double scale_z)
        {
            marker_.scale.z = scale_z;
            return *this;
        }

        Self & points_emplace_back(double x, double y, double z)
        {
            geometry_msgs::Point point;
            point.x = x, point.y = y, point.z = z;

            marker_.points.emplace_back(point);
            return *this;
        }

        Self & points_clear()
        {
            marker_.points.clear();
            return *this;
        }

};