// Generated by gencpp from file motor_pulse/motor4.msg
// DO NOT EDIT!


#ifndef MOTOR_PULSE_MESSAGE_MOTOR4_H
#define MOTOR_PULSE_MESSAGE_MOTOR4_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace motor_pulse
{
template <class ContainerAllocator>
struct motor4_
{
  typedef motor4_<ContainerAllocator> Type;

  motor4_()
    : LF(0)
    , LB(0)
    , RF(0)
    , RB(0)  {
    }
  motor4_(const ContainerAllocator& _alloc)
    : LF(0)
    , LB(0)
    , RF(0)
    , RB(0)  {
  (void)_alloc;
    }



   typedef int32_t _LF_type;
  _LF_type LF;

   typedef int32_t _LB_type;
  _LB_type LB;

   typedef int32_t _RF_type;
  _RF_type RF;

   typedef int32_t _RB_type;
  _RB_type RB;





  typedef boost::shared_ptr< ::motor_pulse::motor4_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::motor_pulse::motor4_<ContainerAllocator> const> ConstPtr;

}; // struct motor4_

typedef ::motor_pulse::motor4_<std::allocator<void> > motor4;

typedef boost::shared_ptr< ::motor_pulse::motor4 > motor4Ptr;
typedef boost::shared_ptr< ::motor_pulse::motor4 const> motor4ConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::motor_pulse::motor4_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::motor_pulse::motor4_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace motor_pulse

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'motor_pulse': ['/home/justin/catkin_ws/src/motor_pulse/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::motor_pulse::motor4_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::motor_pulse::motor4_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motor_pulse::motor4_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::motor_pulse::motor4_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motor_pulse::motor4_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::motor_pulse::motor4_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::motor_pulse::motor4_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d8e5d98ac6cc1583eaa7be12cb299040";
  }

  static const char* value(const ::motor_pulse::motor4_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd8e5d98ac6cc1583ULL;
  static const uint64_t static_value2 = 0xeaa7be12cb299040ULL;
};

template<class ContainerAllocator>
struct DataType< ::motor_pulse::motor4_<ContainerAllocator> >
{
  static const char* value()
  {
    return "motor_pulse/motor4";
  }

  static const char* value(const ::motor_pulse::motor4_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::motor_pulse::motor4_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 LF\n"
"int32 LB\n"
"int32 RF\n"
"int32 RB\n"
;
  }

  static const char* value(const ::motor_pulse::motor4_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::motor_pulse::motor4_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.LF);
      stream.next(m.LB);
      stream.next(m.RF);
      stream.next(m.RB);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct motor4_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::motor_pulse::motor4_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::motor_pulse::motor4_<ContainerAllocator>& v)
  {
    s << indent << "LF: ";
    Printer<int32_t>::stream(s, indent + "  ", v.LF);
    s << indent << "LB: ";
    Printer<int32_t>::stream(s, indent + "  ", v.LB);
    s << indent << "RF: ";
    Printer<int32_t>::stream(s, indent + "  ", v.RF);
    s << indent << "RB: ";
    Printer<int32_t>::stream(s, indent + "  ", v.RB);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MOTOR_PULSE_MESSAGE_MOTOR4_H
