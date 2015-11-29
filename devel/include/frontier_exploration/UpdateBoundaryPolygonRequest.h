// Generated by gencpp from file frontier_exploration/UpdateBoundaryPolygonRequest.msg
// DO NOT EDIT!


#ifndef FRONTIER_EXPLORATION_MESSAGE_UPDATEBOUNDARYPOLYGONREQUEST_H
#define FRONTIER_EXPLORATION_MESSAGE_UPDATEBOUNDARYPOLYGONREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PolygonStamped.h>

namespace frontier_exploration
{
template <class ContainerAllocator>
struct UpdateBoundaryPolygonRequest_
{
  typedef UpdateBoundaryPolygonRequest_<ContainerAllocator> Type;

  UpdateBoundaryPolygonRequest_()
    : explore_boundary()  {
    }
  UpdateBoundaryPolygonRequest_(const ContainerAllocator& _alloc)
    : explore_boundary(_alloc)  {
    }



   typedef  ::geometry_msgs::PolygonStamped_<ContainerAllocator>  _explore_boundary_type;
  _explore_boundary_type explore_boundary;




  typedef boost::shared_ptr< ::frontier_exploration::UpdateBoundaryPolygonRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::frontier_exploration::UpdateBoundaryPolygonRequest_<ContainerAllocator> const> ConstPtr;

}; // struct UpdateBoundaryPolygonRequest_

typedef ::frontier_exploration::UpdateBoundaryPolygonRequest_<std::allocator<void> > UpdateBoundaryPolygonRequest;

typedef boost::shared_ptr< ::frontier_exploration::UpdateBoundaryPolygonRequest > UpdateBoundaryPolygonRequestPtr;
typedef boost::shared_ptr< ::frontier_exploration::UpdateBoundaryPolygonRequest const> UpdateBoundaryPolygonRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::frontier_exploration::UpdateBoundaryPolygonRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::frontier_exploration::UpdateBoundaryPolygonRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace frontier_exploration

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'frontier_exploration': ['/home/loren/Workspaces/final_catkin/src/frontier_exploration/msg', '/home/loren/Workspaces/final_catkin/devel/share/frontier_exploration/msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'visualization_msgs': ['/opt/ros/indigo/share/visualization_msgs/cmake/../msg'], 'move_base_msgs': ['/opt/ros/indigo/share/move_base_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::frontier_exploration::UpdateBoundaryPolygonRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::frontier_exploration::UpdateBoundaryPolygonRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::frontier_exploration::UpdateBoundaryPolygonRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::frontier_exploration::UpdateBoundaryPolygonRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::frontier_exploration::UpdateBoundaryPolygonRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::frontier_exploration::UpdateBoundaryPolygonRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::frontier_exploration::UpdateBoundaryPolygonRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "fcd73e86a17cffec115813ce5a863c84";
  }

  static const char* value(const ::frontier_exploration::UpdateBoundaryPolygonRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xfcd73e86a17cffecULL;
  static const uint64_t static_value2 = 0x115813ce5a863c84ULL;
};

template<class ContainerAllocator>
struct DataType< ::frontier_exploration::UpdateBoundaryPolygonRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "frontier_exploration/UpdateBoundaryPolygonRequest";
  }

  static const char* value(const ::frontier_exploration::UpdateBoundaryPolygonRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::frontier_exploration::UpdateBoundaryPolygonRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/PolygonStamped explore_boundary\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PolygonStamped\n\
# This represents a Polygon with reference coordinate frame and timestamp\n\
Header header\n\
Polygon polygon\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Polygon\n\
#A specification of a polygon where the first and last points are assumed to be connected\n\
Point32[] points\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point32\n\
# This contains the position of a point in free space(with 32 bits of precision).\n\
# It is recommeded to use Point wherever possible instead of Point32.  \n\
# \n\
# This recommendation is to promote interoperability.  \n\
#\n\
# This message is designed to take up less space when sending\n\
# lots of points at once, as in the case of a PointCloud.  \n\
\n\
float32 x\n\
float32 y\n\
float32 z\n\
";
  }

  static const char* value(const ::frontier_exploration::UpdateBoundaryPolygonRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::frontier_exploration::UpdateBoundaryPolygonRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.explore_boundary);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct UpdateBoundaryPolygonRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::frontier_exploration::UpdateBoundaryPolygonRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::frontier_exploration::UpdateBoundaryPolygonRequest_<ContainerAllocator>& v)
  {
    s << indent << "explore_boundary: ";
    s << std::endl;
    Printer< ::geometry_msgs::PolygonStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.explore_boundary);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FRONTIER_EXPLORATION_MESSAGE_UPDATEBOUNDARYPOLYGONREQUEST_H
