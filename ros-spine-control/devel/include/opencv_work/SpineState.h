// Generated by gencpp from file opencv_work/SpineState.msg
// DO NOT EDIT!


#ifndef OPENCV_WORK_MESSAGE_SPINESTATE_H
#define OPENCV_WORK_MESSAGE_SPINESTATE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace opencv_work
{
template <class ContainerAllocator>
struct SpineState_
{
  typedef SpineState_<ContainerAllocator> Type;

  SpineState_()
    : rotation(0.0)
    , comy(0.0)
    , comx(0.0)  {
    }
  SpineState_(const ContainerAllocator& _alloc)
    : rotation(0.0)
    , comy(0.0)
    , comx(0.0)  {
  (void)_alloc;
    }



   typedef double _rotation_type;
  _rotation_type rotation;

   typedef double _comy_type;
  _comy_type comy;

   typedef double _comx_type;
  _comx_type comx;





  typedef boost::shared_ptr< ::opencv_work::SpineState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::opencv_work::SpineState_<ContainerAllocator> const> ConstPtr;

}; // struct SpineState_

typedef ::opencv_work::SpineState_<std::allocator<void> > SpineState;

typedef boost::shared_ptr< ::opencv_work::SpineState > SpineStatePtr;
typedef boost::shared_ptr< ::opencv_work::SpineState const> SpineStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::opencv_work::SpineState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::opencv_work::SpineState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::opencv_work::SpineState_<ContainerAllocator1> & lhs, const ::opencv_work::SpineState_<ContainerAllocator2> & rhs)
{
  return lhs.rotation == rhs.rotation &&
    lhs.comy == rhs.comy &&
    lhs.comx == rhs.comx;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::opencv_work::SpineState_<ContainerAllocator1> & lhs, const ::opencv_work::SpineState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace opencv_work

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::opencv_work::SpineState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::opencv_work::SpineState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::opencv_work::SpineState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::opencv_work::SpineState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::opencv_work::SpineState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::opencv_work::SpineState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::opencv_work::SpineState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "362faa163ba6b21bd1aa0295e7ccf8ab";
  }

  static const char* value(const ::opencv_work::SpineState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x362faa163ba6b21bULL;
  static const uint64_t static_value2 = 0xd1aa0295e7ccf8abULL;
};

template<class ContainerAllocator>
struct DataType< ::opencv_work::SpineState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "opencv_work/SpineState";
  }

  static const char* value(const ::opencv_work::SpineState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::opencv_work::SpineState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 rotation\n"
"float64 comy\n"
"float64 comx\n"
;
  }

  static const char* value(const ::opencv_work::SpineState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::opencv_work::SpineState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.rotation);
      stream.next(m.comy);
      stream.next(m.comx);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SpineState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::opencv_work::SpineState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::opencv_work::SpineState_<ContainerAllocator>& v)
  {
    s << indent << "rotation: ";
    Printer<double>::stream(s, indent + "  ", v.rotation);
    s << indent << "comy: ";
    Printer<double>::stream(s, indent + "  ", v.comy);
    s << indent << "comx: ";
    Printer<double>::stream(s, indent + "  ", v.comx);
  }
};

} // namespace message_operations
} // namespace ros

#endif // OPENCV_WORK_MESSAGE_SPINESTATE_H
