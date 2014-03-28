/* Auto-generated by genmsg_cpp for file /home/cody/groovy_workspace/GitHubRepos/X-Bar_stack/SpiderRobot_pkg/msg/My2Num.msg */
#ifndef SPIDERROBOT_PKG_MESSAGE_MY2NUM_H
#define SPIDERROBOT_PKG_MESSAGE_MY2NUM_H
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


namespace SpiderRobot_pkg
{
template <class ContainerAllocator>
struct My2Num_ {
  typedef My2Num_<ContainerAllocator> Type;

  My2Num_()
  : cha(0)
  , pos(0)
  {
  }

  My2Num_(const ContainerAllocator& _alloc)
  : cha(0)
  , pos(0)
  {
  }

  typedef int8_t _cha_type;
  int8_t cha;

  typedef int8_t _pos_type;
  int8_t pos;


  typedef boost::shared_ptr< ::SpiderRobot_pkg::My2Num_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::SpiderRobot_pkg::My2Num_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct My2Num
typedef  ::SpiderRobot_pkg::My2Num_<std::allocator<void> > My2Num;

typedef boost::shared_ptr< ::SpiderRobot_pkg::My2Num> My2NumPtr;
typedef boost::shared_ptr< ::SpiderRobot_pkg::My2Num const> My2NumConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::SpiderRobot_pkg::My2Num_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::SpiderRobot_pkg::My2Num_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace SpiderRobot_pkg

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::SpiderRobot_pkg::My2Num_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::SpiderRobot_pkg::My2Num_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::SpiderRobot_pkg::My2Num_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b11b2efb605ac36c546d47d65708d2c8";
  }

  static const char* value(const  ::SpiderRobot_pkg::My2Num_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xb11b2efb605ac36cULL;
  static const uint64_t static_value2 = 0x546d47d65708d2c8ULL;
};

template<class ContainerAllocator>
struct DataType< ::SpiderRobot_pkg::My2Num_<ContainerAllocator> > {
  static const char* value() 
  {
    return "SpiderRobot_pkg/My2Num";
  }

  static const char* value(const  ::SpiderRobot_pkg::My2Num_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::SpiderRobot_pkg::My2Num_<ContainerAllocator> > {
  static const char* value() 
  {
    return "int8 cha\n\
int8 pos\n\
\n\
";
  }

  static const char* value(const  ::SpiderRobot_pkg::My2Num_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::SpiderRobot_pkg::My2Num_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::SpiderRobot_pkg::My2Num_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.cha);
    stream.next(m.pos);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct My2Num_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::SpiderRobot_pkg::My2Num_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::SpiderRobot_pkg::My2Num_<ContainerAllocator> & v) 
  {
    s << indent << "cha: ";
    Printer<int8_t>::stream(s, indent + "  ", v.cha);
    s << indent << "pos: ";
    Printer<int8_t>::stream(s, indent + "  ", v.pos);
  }
};


} // namespace message_operations
} // namespace ros

#endif // SPIDERROBOT_PKG_MESSAGE_MY2NUM_H
