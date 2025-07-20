// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from libo_interfaces:msg/BookInfo.idl
// generated code does not contain a copyright notice

#ifndef LIBO_INTERFACES__MSG__DETAIL__BOOK_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define LIBO_INTERFACES__MSG__DETAIL__BOOK_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include <cstddef>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "libo_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "libo_interfaces/msg/detail/book_info__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace libo_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_libo_interfaces
cdr_serialize(
  const libo_interfaces::msg::BookInfo & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_libo_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  libo_interfaces::msg::BookInfo & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_libo_interfaces
get_serialized_size(
  const libo_interfaces::msg::BookInfo & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_libo_interfaces
max_serialized_size_BookInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_libo_interfaces
cdr_serialize_key(
  const libo_interfaces::msg::BookInfo & ros_message,
  eprosima::fastcdr::Cdr &);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_libo_interfaces
get_serialized_size_key(
  const libo_interfaces::msg::BookInfo & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_libo_interfaces
max_serialized_size_key_BookInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace libo_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_libo_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, libo_interfaces, msg, BookInfo)();

#ifdef __cplusplus
}
#endif

#endif  // LIBO_INTERFACES__MSG__DETAIL__BOOK_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
