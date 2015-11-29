// Generated by gencpp from file frontier_exploration/ExploreTaskActionGoal.msg
// DO NOT EDIT!


#ifndef FRONTIER_EXPLORATION_MESSAGE_EXPLORETASKACTIONGOAL_H
#define FRONTIER_EXPLORATION_MESSAGE_EXPLORETASKACTIONGOAL_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>
#include <actionlib_msgs/GoalID.h>
#include <frontier_exploration/ExploreTaskGoal.h>

namespace frontier_exploration
{
template <class ContainerAllocator>
struct ExploreTaskActionGoal_
{
  typedef ExploreTaskActionGoal_<ContainerAllocator> Type;

  ExploreTaskActionGoal_()
    : header()
    , goal_id()
    , goal()  {
    }
  ExploreTaskActionGoal_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , goal_id(_alloc)
    , goal(_alloc)  {
    }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef  ::actionlib_msgs::GoalID_<ContainerAllocator>  _goal_id_type;
  _goal_id_type goal_id;

   typedef  ::frontier_exploration::ExploreTaskGoal_<ContainerAllocator>  _goal_type;
  _goal_type goal;




  typedef boost::shared_ptr< ::frontier_exploration::ExploreTaskActionGoal_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::frontier_exploration::ExploreTaskActionGoal_<ContainerAllocator> const> ConstPtr;

}; // struct ExploreTaskActionGoal_

typedef ::frontier_exploration::ExploreTaskActionGoal_<std::allocator<void> > ExploreTaskActionGoal;

typedef boost::shared_ptr< ::frontier_exploration::ExploreTaskActionGoal > ExploreTaskActionGoalPtr;
typedef boost::shared_ptr< ::frontier_exploration::ExploreTaskActionGoal const> ExploreTaskActionGoalConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::frontier_exploration::ExploreTaskActionGoal_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::frontier_exploration::ExploreTaskActionGoal_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace frontier_exploration

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'frontier_exploration': ['/home/loren/Workspaces/final_catkin/src/frontier_exploration/msg', '/home/loren/Workspaces/final_catkin/devel/share/frontier_exploration/msg'], 'actionlib_msgs': ['/opt/ros/indigo/share/actionlib_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'visualization_msgs': ['/opt/ros/indigo/share/visualization_msgs/cmake/../msg'], 'move_base_msgs': ['/opt/ros/indigo/share/move_base_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::frontier_exploration::ExploreTaskActionGoal_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::frontier_exploration::ExploreTaskActionGoal_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::frontier_exploration::ExploreTaskActionGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::frontier_exploration::ExploreTaskActionGoal_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::frontier_exploration::ExploreTaskActionGoal_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::frontier_exploration::ExploreTaskActionGoal_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::frontier_exploration::ExploreTaskActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "9636099f4dce4531a0248d068494bc98";
  }

  static const char* value(const ::frontier_exploration::ExploreTaskActionGoal_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x9636099f4dce4531ULL;
  static const uint64_t static_value2 = 0xa0248d068494bc98ULL;
};

template<class ContainerAllocator>
struct DataType< ::frontier_exploration::ExploreTaskActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "frontier_exploration/ExploreTaskActionGoal";
  }

  static const char* value(const ::frontier_exploration::ExploreTaskActionGoal_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::frontier_exploration::ExploreTaskActionGoal_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
ExploreTaskGoal goal\n\
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
MSG: actionlib_msgs/GoalID\n\
# The stamp should store the time at which this goal was requested.\n\
# It is used by an action server when it tries to preempt all\n\
# goals that were requested before a certain time\n\
time stamp\n\
\n\
# The id provides a way to associate feedback and\n\
# result message with specific goal requests. The id\n\
# specified must be unique.\n\
string id\n\
\n\
\n\
================================================================================\n\
MSG: frontier_exploration/ExploreTaskGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#Boundary for frontier exploration\n\
geometry_msgs/PolygonStamped explore_boundary\n\
#Center point for frontier exploration, inside explore_boundary\n\
geometry_msgs/PointStamped explore_center\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PolygonStamped\n\
# This represents a Polygon with reference coordinate frame and timestamp\n\
Header header\n\
Polygon polygon\n\
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
================================================================================\n\
MSG: geometry_msgs/PointStamped\n\
# This represents a Point with reference coordinate frame and timestamp\n\
Header header\n\
Point point\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
";
  }

  static const char* value(const ::frontier_exploration::ExploreTaskActionGoal_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::frontier_exploration::ExploreTaskActionGoal_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.goal_id);
      stream.next(m.goal);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct ExploreTaskActionGoal_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::frontier_exploration::ExploreTaskActionGoal_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::frontier_exploration::ExploreTaskActionGoal_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "goal_id: ";
    s << std::endl;
    Printer< ::actionlib_msgs::GoalID_<ContainerAllocator> >::stream(s, indent + "  ", v.goal_id);
    s << indent << "goal: ";
    s << std::endl;
    Printer< ::frontier_exploration::ExploreTaskGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.goal);
  }
};

} // namespace message_operations
} // namespace ros

#endif // FRONTIER_EXPLORATION_MESSAGE_EXPLORETASKACTIONGOAL_H
