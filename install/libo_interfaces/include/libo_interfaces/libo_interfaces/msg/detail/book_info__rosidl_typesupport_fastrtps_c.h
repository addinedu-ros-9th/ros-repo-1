// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from libo_interfaces:msg/BookInfo.idl
// generated code does not contain a copyright notice
#ifndef LIBO_INTERFACES__MSG__DETAIL__BOOK_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define LIBO_INTERFACES__MSG__DETAIL__BOOK_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "libo_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "libo_interfaces/msg/detail/book_info__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_libo_interfaces
bool cdr_serialize_libo_interfaces__msg__BookInfo(
  const libo_interfaces__msg__BookInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_libo_interfaces
bool cdr_deserialize_libo_interfaces__msg__BookInfo(
  eprosima::fastcdr::Cdr &,
  libo_interfaces__msg__BookInfo * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_libo_interfaces
size_t get_serialized_size_libo_interfaces__msg__BookInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_libo_interfaces
size_t max_serialized_size_libo_interfaces__msg__BookInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_libo_interfaces
bool cdr_serialize_key_libo_interfaces__msg__BookInfo(
  const libo_interfaces__msg__BookInfo * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_libo_interfaces
size_t get_serialized_size_key_libo_interfaces__msg__BookInfo(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_libo_interfaces
size_t max_serialized_size_key_libo_interfaces__msg__BookInfo(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_libo_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, libo_interfaces, msg, BookInfo)();

#ifdef __cplusplus
}
#endif

#endif  // LIBO_INTERFACES__MSG__DETAIL__BOOK_INFO__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
