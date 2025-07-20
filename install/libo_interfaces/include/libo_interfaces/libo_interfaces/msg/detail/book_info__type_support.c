// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from libo_interfaces:msg/BookInfo.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "libo_interfaces/msg/detail/book_info__rosidl_typesupport_introspection_c.h"
#include "libo_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "libo_interfaces/msg/detail/book_info__functions.h"
#include "libo_interfaces/msg/detail/book_info__struct.h"


// Include directives for member types
// Member `title`
// Member `author`
// Member `publisher`
// Member `category_name`
// Member `location`
// Member `isbn`
// Member `cover_image_url`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void libo_interfaces__msg__BookInfo__rosidl_typesupport_introspection_c__BookInfo_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  libo_interfaces__msg__BookInfo__init(message_memory);
}

void libo_interfaces__msg__BookInfo__rosidl_typesupport_introspection_c__BookInfo_fini_function(void * message_memory)
{
  libo_interfaces__msg__BookInfo__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember libo_interfaces__msg__BookInfo__rosidl_typesupport_introspection_c__BookInfo_message_member_array[10] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(libo_interfaces__msg__BookInfo, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "title",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(libo_interfaces__msg__BookInfo, title),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "author",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(libo_interfaces__msg__BookInfo, author),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "publisher",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(libo_interfaces__msg__BookInfo, publisher),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "category_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(libo_interfaces__msg__BookInfo, category_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "location",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(libo_interfaces__msg__BookInfo, location),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "price",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(libo_interfaces__msg__BookInfo, price),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "stock_quantity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(libo_interfaces__msg__BookInfo, stock_quantity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "isbn",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(libo_interfaces__msg__BookInfo, isbn),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "cover_image_url",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(libo_interfaces__msg__BookInfo, cover_image_url),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers libo_interfaces__msg__BookInfo__rosidl_typesupport_introspection_c__BookInfo_message_members = {
  "libo_interfaces__msg",  // message namespace
  "BookInfo",  // message name
  10,  // number of fields
  sizeof(libo_interfaces__msg__BookInfo),
  false,  // has_any_key_member_
  libo_interfaces__msg__BookInfo__rosidl_typesupport_introspection_c__BookInfo_message_member_array,  // message members
  libo_interfaces__msg__BookInfo__rosidl_typesupport_introspection_c__BookInfo_init_function,  // function to initialize message memory (memory has to be allocated)
  libo_interfaces__msg__BookInfo__rosidl_typesupport_introspection_c__BookInfo_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t libo_interfaces__msg__BookInfo__rosidl_typesupport_introspection_c__BookInfo_message_type_support_handle = {
  0,
  &libo_interfaces__msg__BookInfo__rosidl_typesupport_introspection_c__BookInfo_message_members,
  get_message_typesupport_handle_function,
  &libo_interfaces__msg__BookInfo__get_type_hash,
  &libo_interfaces__msg__BookInfo__get_type_description,
  &libo_interfaces__msg__BookInfo__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_libo_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, libo_interfaces, msg, BookInfo)() {
  if (!libo_interfaces__msg__BookInfo__rosidl_typesupport_introspection_c__BookInfo_message_type_support_handle.typesupport_identifier) {
    libo_interfaces__msg__BookInfo__rosidl_typesupport_introspection_c__BookInfo_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &libo_interfaces__msg__BookInfo__rosidl_typesupport_introspection_c__BookInfo_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
