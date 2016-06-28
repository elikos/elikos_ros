/* Auto-generated by genmsg_cpp for file /home/alessio/RosWorkspace/sandbox/action_controller/msg/MultiDofFollowJointTrajectoryFeedback.msg */
#ifndef ACTION_CONTROLLER_MESSAGE_MULTIDOFFOLLOWJOINTTRAJECTORYFEEDBACK_H
#define ACTION_CONTROLLER_MESSAGE_MULTIDOFFOLLOWJOINTTRAJECTORYFEEDBACK_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "std_msgs/Header.h"
#include "trajectory_msgs/MultiDOFJointTrajectoryPoint.h"

namespace action_controller
{
template <class ContainerAllocator>
struct MultiDofFollowJointTrajectoryFeedback_ {
  typedef MultiDofFollowJointTrajectoryFeedback_<ContainerAllocator> Type;

  MultiDofFollowJointTrajectoryFeedback_()
  : header()
  , joint_names()
  , desired()
  , actual()
  , error()
  {
  }

  MultiDofFollowJointTrajectoryFeedback_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , joint_names(_alloc)
  , desired(_alloc)
  , actual(_alloc)
  , error(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  _joint_names_type;
  std::vector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > , typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::other >  joint_names;

  typedef  ::trajectory_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator>  _desired_type;
   ::trajectory_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator>  desired;

  typedef  ::trajectory_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator>  _actual_type;
   ::trajectory_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator>  actual;

  typedef  ::trajectory_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator>  _error_type;
   ::trajectory_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator>  error;


  typedef boost::shared_ptr< ::action_controller::MultiDofFollowJointTrajectoryFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::action_controller::MultiDofFollowJointTrajectoryFeedback_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct MultiDofFollowJointTrajectoryFeedback
typedef  ::action_controller::MultiDofFollowJointTrajectoryFeedback_<std::allocator<void> > MultiDofFollowJointTrajectoryFeedback;

typedef boost::shared_ptr< ::action_controller::MultiDofFollowJointTrajectoryFeedback> MultiDofFollowJointTrajectoryFeedbackPtr;
typedef boost::shared_ptr< ::action_controller::MultiDofFollowJointTrajectoryFeedback const> MultiDofFollowJointTrajectoryFeedbackConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::action_controller::MultiDofFollowJointTrajectoryFeedback_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::action_controller::MultiDofFollowJointTrajectoryFeedback_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace action_controller

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::action_controller::MultiDofFollowJointTrajectoryFeedback_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::action_controller::MultiDofFollowJointTrajectoryFeedback_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::action_controller::MultiDofFollowJointTrajectoryFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "4d0c4cf9e9eeebe573320d2109a20899";
  }

  static const char* value(const  ::action_controller::MultiDofFollowJointTrajectoryFeedback_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x4d0c4cf9e9eeebe5ULL;
  static const uint64_t static_value2 = 0x73320d2109a20899ULL;
};

template<class ContainerAllocator>
struct DataType< ::action_controller::MultiDofFollowJointTrajectoryFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "action_controller/MultiDofFollowJointTrajectoryFeedback";
  }

  static const char* value(const  ::action_controller::MultiDofFollowJointTrajectoryFeedback_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::action_controller::MultiDofFollowJointTrajectoryFeedback_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
Header header\n\
string[] joint_names\n\
trajectory_msgs/MultiDOFJointTrajectoryPoint desired\n\
trajectory_msgs/MultiDOFJointTrajectoryPoint actual\n\
trajectory_msgs/MultiDOFJointTrajectoryPoint error\n\
\n\
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
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: trajectory_msgs/MultiDOFJointTrajectoryPoint\n\
geometry_msgs/Transform[] transforms\n\
duration time_from_start\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Transform\n\
# This represents the transform between two coordinate frames in free space.\n\
\n\
Vector3 translation\n\
Quaternion rotation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
";
  }

  static const char* value(const  ::action_controller::MultiDofFollowJointTrajectoryFeedback_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::action_controller::MultiDofFollowJointTrajectoryFeedback_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::action_controller::MultiDofFollowJointTrajectoryFeedback_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::action_controller::MultiDofFollowJointTrajectoryFeedback_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.joint_names);
    stream.next(m.desired);
    stream.next(m.actual);
    stream.next(m.error);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct MultiDofFollowJointTrajectoryFeedback_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::action_controller::MultiDofFollowJointTrajectoryFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::action_controller::MultiDofFollowJointTrajectoryFeedback_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "joint_names[]" << std::endl;
    for (size_t i = 0; i < v.joint_names.size(); ++i)
    {
      s << indent << "  joint_names[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.joint_names[i]);
    }
    s << indent << "desired: ";
s << std::endl;
    Printer< ::trajectory_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator> >::stream(s, indent + "  ", v.desired);
    s << indent << "actual: ";
s << std::endl;
    Printer< ::trajectory_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator> >::stream(s, indent + "  ", v.actual);
    s << indent << "error: ";
s << std::endl;
    Printer< ::trajectory_msgs::MultiDOFJointTrajectoryPoint_<ContainerAllocator> >::stream(s, indent + "  ", v.error);
  }
};


} // namespace message_operations
} // namespace ros

#endif // ACTION_CONTROLLER_MESSAGE_MULTIDOFFOLLOWJOINTTRAJECTORYFEEDBACK_H

