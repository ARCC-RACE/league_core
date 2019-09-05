// Generated by gencpp from file deepracer_simulation/GetWaypointSrvResponse.msg
// DO NOT EDIT!


#ifndef DEEPRACER_SIMULATION_MESSAGE_GETWAYPOINTSRVRESPONSE_H
#define DEEPRACER_SIMULATION_MESSAGE_GETWAYPOINTSRVRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace deepracer_simulation
{
template <class ContainerAllocator>
struct GetWaypointSrvResponse_
{
  typedef GetWaypointSrvResponse_<ContainerAllocator> Type;

  GetWaypointSrvResponse_()
    : row(0)
    , col(0)
    , waypoints()  {
    }
  GetWaypointSrvResponse_(const ContainerAllocator& _alloc)
    : row(0)
    , col(0)
    , waypoints(_alloc)  {
  (void)_alloc;
    }



   typedef int32_t _row_type;
  _row_type row;

   typedef int32_t _col_type;
  _col_type col;

   typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _waypoints_type;
  _waypoints_type waypoints;





  typedef boost::shared_ptr< ::deepracer_simulation::GetWaypointSrvResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::deepracer_simulation::GetWaypointSrvResponse_<ContainerAllocator> const> ConstPtr;

}; // struct GetWaypointSrvResponse_

typedef ::deepracer_simulation::GetWaypointSrvResponse_<std::allocator<void> > GetWaypointSrvResponse;

typedef boost::shared_ptr< ::deepracer_simulation::GetWaypointSrvResponse > GetWaypointSrvResponsePtr;
typedef boost::shared_ptr< ::deepracer_simulation::GetWaypointSrvResponse const> GetWaypointSrvResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::deepracer_simulation::GetWaypointSrvResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::deepracer_simulation::GetWaypointSrvResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace deepracer_simulation

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::deepracer_simulation::GetWaypointSrvResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::deepracer_simulation::GetWaypointSrvResponse_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::deepracer_simulation::GetWaypointSrvResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::deepracer_simulation::GetWaypointSrvResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::deepracer_simulation::GetWaypointSrvResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::deepracer_simulation::GetWaypointSrvResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::deepracer_simulation::GetWaypointSrvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "dd1c3c0f312afb554365a4b5e8f07a10";
  }

  static const char* value(const ::deepracer_simulation::GetWaypointSrvResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xdd1c3c0f312afb55ULL;
  static const uint64_t static_value2 = 0x4365a4b5e8f07a10ULL;
};

template<class ContainerAllocator>
struct DataType< ::deepracer_simulation::GetWaypointSrvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "deepracer_simulation/GetWaypointSrvResponse";
  }

  static const char* value(const ::deepracer_simulation::GetWaypointSrvResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::deepracer_simulation::GetWaypointSrvResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 row\n"
"int32 col\n"
"float64[] waypoints\n"
;
  }

  static const char* value(const ::deepracer_simulation::GetWaypointSrvResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::deepracer_simulation::GetWaypointSrvResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.row);
      stream.next(m.col);
      stream.next(m.waypoints);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct GetWaypointSrvResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::deepracer_simulation::GetWaypointSrvResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::deepracer_simulation::GetWaypointSrvResponse_<ContainerAllocator>& v)
  {
    s << indent << "row: ";
    Printer<int32_t>::stream(s, indent + "  ", v.row);
    s << indent << "col: ";
    Printer<int32_t>::stream(s, indent + "  ", v.col);
    s << indent << "waypoints[]" << std::endl;
    for (size_t i = 0; i < v.waypoints.size(); ++i)
    {
      s << indent << "  waypoints[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.waypoints[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // DEEPRACER_SIMULATION_MESSAGE_GETWAYPOINTSRVRESPONSE_H
