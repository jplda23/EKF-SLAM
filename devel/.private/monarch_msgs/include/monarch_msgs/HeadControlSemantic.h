// Generated by gencpp from file monarch_msgs/HeadControlSemantic.msg
// DO NOT EDIT!


#ifndef MONARCH_MSGS_MESSAGE_HEADCONTROLSEMANTIC_H
#define MONARCH_MSGS_MESSAGE_HEADCONTROLSEMANTIC_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace monarch_msgs
{
template <class ContainerAllocator>
struct HeadControlSemantic_
{
  typedef HeadControlSemantic_<ContainerAllocator> Type;

  HeadControlSemantic_()
    : cardinal_direction()
    , speed(0)  {
    }
  HeadControlSemantic_(const ContainerAllocator& _alloc)
    : cardinal_direction(_alloc)
    , speed(0)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _cardinal_direction_type;
  _cardinal_direction_type cardinal_direction;

   typedef uint8_t _speed_type;
  _speed_type speed;



  enum {
    NORMAL = 0u,
    SLOW = 1u,
    FAST = 2u,
  };


  typedef boost::shared_ptr< ::monarch_msgs::HeadControlSemantic_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::monarch_msgs::HeadControlSemantic_<ContainerAllocator> const> ConstPtr;

}; // struct HeadControlSemantic_

typedef ::monarch_msgs::HeadControlSemantic_<std::allocator<void> > HeadControlSemantic;

typedef boost::shared_ptr< ::monarch_msgs::HeadControlSemantic > HeadControlSemanticPtr;
typedef boost::shared_ptr< ::monarch_msgs::HeadControlSemantic const> HeadControlSemanticConstPtr;

// constants requiring out of line definition

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::monarch_msgs::HeadControlSemantic_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::monarch_msgs::HeadControlSemantic_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace monarch_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'monarch_msgs': ['/home/de/catkin_ws/src/mbot_simulation_sa/resources/packages/monarch_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::monarch_msgs::HeadControlSemantic_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::monarch_msgs::HeadControlSemantic_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::monarch_msgs::HeadControlSemantic_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::monarch_msgs::HeadControlSemantic_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::monarch_msgs::HeadControlSemantic_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::monarch_msgs::HeadControlSemantic_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::monarch_msgs::HeadControlSemantic_<ContainerAllocator> >
{
  static const char* value()
  {
    return "a8f7a062da2d04ec1d87e1a59f64f6af";
  }

  static const char* value(const ::monarch_msgs::HeadControlSemantic_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xa8f7a062da2d04ecULL;
  static const uint64_t static_value2 = 0x1d87e1a59f64f6afULL;
};

template<class ContainerAllocator>
struct DataType< ::monarch_msgs::HeadControlSemantic_<ContainerAllocator> >
{
  static const char* value()
  {
    return "monarch_msgs/HeadControlSemantic";
  }

  static const char* value(const ::monarch_msgs::HeadControlSemantic_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::monarch_msgs::HeadControlSemantic_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# If you add more, please do not break the current definitions\n\
uint8 NORMAL=0\n\
uint8 SLOW=1\n\
uint8 FAST=2\n\
\n\
# Example for NW, accepts:\n\
# {NW, NORTHEAST}\n\
string cardinal_direction\n\
uint8 speed\n\
";
  }

  static const char* value(const ::monarch_msgs::HeadControlSemantic_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::monarch_msgs::HeadControlSemantic_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.cardinal_direction);
      stream.next(m.speed);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct HeadControlSemantic_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::monarch_msgs::HeadControlSemantic_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::monarch_msgs::HeadControlSemantic_<ContainerAllocator>& v)
  {
    s << indent << "cardinal_direction: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.cardinal_direction);
    s << indent << "speed: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.speed);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MONARCH_MSGS_MESSAGE_HEADCONTROLSEMANTIC_H
