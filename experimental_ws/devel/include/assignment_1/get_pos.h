// Generated by gencpp from file assignment_1/get_pos.msg
// DO NOT EDIT!


#ifndef ASSIGNMENT_1_MESSAGE_GET_POS_H
#define ASSIGNMENT_1_MESSAGE_GET_POS_H

#include <ros/service_traits.h>


#include <assignment_1/get_posRequest.h>
#include <assignment_1/get_posResponse.h>


namespace assignment_1
{

struct get_pos
{

typedef get_posRequest Request;
typedef get_posResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct get_pos
} // namespace assignment_1


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::assignment_1::get_pos > {
  static const char* value()
  {
    return "689b3c9d7a6b02bd3b9a51ce8fe45a3b";
  }

  static const char* value(const ::assignment_1::get_pos&) { return value(); }
};

template<>
struct DataType< ::assignment_1::get_pos > {
  static const char* value()
  {
    return "assignment_1/get_pos";
  }

  static const char* value(const ::assignment_1::get_pos&) { return value(); }
};


// service_traits::MD5Sum< ::assignment_1::get_posRequest> should match 
// service_traits::MD5Sum< ::assignment_1::get_pos > 
template<>
struct MD5Sum< ::assignment_1::get_posRequest>
{
  static const char* value()
  {
    return MD5Sum< ::assignment_1::get_pos >::value();
  }
  static const char* value(const ::assignment_1::get_posRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::assignment_1::get_posRequest> should match 
// service_traits::DataType< ::assignment_1::get_pos > 
template<>
struct DataType< ::assignment_1::get_posRequest>
{
  static const char* value()
  {
    return DataType< ::assignment_1::get_pos >::value();
  }
  static const char* value(const ::assignment_1::get_posRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::assignment_1::get_posResponse> should match 
// service_traits::MD5Sum< ::assignment_1::get_pos > 
template<>
struct MD5Sum< ::assignment_1::get_posResponse>
{
  static const char* value()
  {
    return MD5Sum< ::assignment_1::get_pos >::value();
  }
  static const char* value(const ::assignment_1::get_posResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::assignment_1::get_posResponse> should match 
// service_traits::DataType< ::assignment_1::get_pos > 
template<>
struct DataType< ::assignment_1::get_posResponse>
{
  static const char* value()
  {
    return DataType< ::assignment_1::get_pos >::value();
  }
  static const char* value(const ::assignment_1::get_posResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ASSIGNMENT_1_MESSAGE_GET_POS_H
