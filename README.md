# RViz Basic Markers

This repository demonstarte the usage of rviz basic markers. This ROS package will display all the available markers including your own mesh in RViz. For more information about markers please visit this [page](http://wiki.ros.org/rviz/DisplayTypes/Marker).  

## Marker Usage

Parameters that all markers share.  
```cpp
// Set relative frame and time
marker_msg_.header.frame_id = "map";    // Learn more about it at tf
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
```

Parameters specifically for each Markers.  

### Arrow (0)

```cpp
// Method 1 of defining an arrow
// =============================
// Set marker scale
marker_msg_.scale.x = 0.1;      // Shaft diameter
marker_msg_.scale.y = 0.2;      // Head diameter
marker_msg_.scale.z = 0.1;      // Head length
// Set start and end point at(0) and at(1)
point_.x = 0.0, point_.y = 0.0, point_.z = 0.0;
marker_msg_.points.push_back(point_);
point_.x = 1.0, point_.y = 1.0, point_.z = 0.0;
marker_msg_.points.push_back(point_);
// Set marker shape
marker_msg_.type = visualization_msgs::Marker::ARROW;

// Method 2 of defining an arrow
// =============================
// Set marker scale
marker_msg_.scale.x = 1.0;      // Arrow length
marker_msg_.scale.y = 0.1;      // Arrow width
marker_msg_.scale.z = 0.1;      // Arrow height
// Set marker shape
marker_msg_.type = visualization_msgs::Marker::ARROW;
```

### Cube (1)

```cpp
// Set marker scale -- 1x1x1 here means 1m on each side
marker_msg_.scale.x = 1.0;
marker_msg_.scale.y = 1.0;
marker_msg_.scale.z = 1.0;
// Set marker shape
marker_msg_.type = visualization_msgs::Marker::CUBE;
```

### SPHERE (2)

```cpp
marker_msg_.scale.x = 1.0;
marker_msg_.scale.y = 1.0;
marker_msg_.scale.z = 1.0;
// Set marker shape
marker_msg_.type = visualization_msgs::Marker::SPHERE;
```

### CYLINDER (3)

By setting x and y scale differently you can get an ellipse.  
```cpp
// Set marker scale
marker_msg_.scale.x = 1.0;      // Diameter in x direction
marker_msg_.scale.y = 1.0;      // Diameter in y direction
marker_msg_.scale.z = 1.0;      // Cylinder height
// Set marker shape
marker_msg_.type = visualization_msgs::Marker::CYLINDER;
```

### LINE_STRIP (4)

Note that pose is still used (the points in the line will be transformed by them), and the lines will be correct relative to the frame id specified in the header.  
```cpp
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
// Set marker shape
marker_msg_.type = visualization_msgs::Marker::LINE_STRIP;
```

### LINE_LIST

Note that pose is still used (the points in the line will be transformed by them), and the lines will be correct relative to the frame id specified in the header.  
```cpp
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
// Set marker shape
marker_msg_.type = visualization_msgs::Marker::LINE_LIST;
```

### CUBE_LIST (6)

Pose is not used.  
```cpp
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
// Set marker shape
marker_msg_.type = visualization_msgs::Marker::CUBE_LIST;
```

### SPHERE_LIST (7)

Note that pose is still used (the points in the line will be transformed by them), and the lines will be correct relative to the frame id specified in the header.  
```cpp
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
// Set marker shape
marker_msg_.type = visualization_msgs::Marker::SPHERE_LIST;
```

### POINTS (8)

```cpp
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
// Set marker shape
marker_msg_.type = visualization_msgs::Marker::POINTS;
```

## Clear Marker Message

Remember to clear marker msg before using it for the second time as remnants may cause unwanted behaviour.  
```cpp
// Before use marker msg again, perform clean up
marker_msg_.points.clear();
marker_msg_.scale.x = 0.0;
marker_msg_.scale.y = 0.0;
marker_msg_.scale.z = 0.0;
```

## visualization_msgs/Marker Message

```yaml
# See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz

uint8 ARROW=0
uint8 CUBE=1
uint8 SPHERE=2
uint8 CYLINDER=3
uint8 LINE_STRIP=4
uint8 LINE_LIST=5
uint8 CUBE_LIST=6
uint8 SPHERE_LIST=7
uint8 POINTS=8
uint8 TEXT_VIEW_FACING=9
uint8 MESH_RESOURCE=10
uint8 TRIANGLE_LIST=11

uint8 ADD=0
uint8 MODIFY=0
uint8 DELETE=2
uint8 DELETEALL=3

Header header                        # header for time/frame information
string ns                            # Namespace to place this object in... used in conjunction with id to create a unique name for the object
int32 id                           # object ID useful in conjunction with the namespace for manipulating and deleting the object later
int32 type                         # Type of object
int32 action                         # 0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
geometry_msgs/Pose pose                 # Pose of the object
geometry_msgs/Vector3 scale             # Scale of the object 1,1,1 means default (usually 1 meter square)
std_msgs/ColorRGBA color             # Color [0.0-1.0]
duration lifetime                    # How long the object should last before being automatically deleted.  0 means forever
bool frame_locked                    # If this marker should be frame-locked, i.e. retransformed into its frame every timestep

#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
geometry_msgs/Point[] points
#Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
#number of colors must either be 0 or equal to the number of points
#NOTE: alpha is not yet used
std_msgs/ColorRGBA[] colors

# NOTE: only used for text markers
string text

# NOTE: only used for MESH_RESOURCE markers
string mesh_resource
bool mesh_use_embedded_materials

```