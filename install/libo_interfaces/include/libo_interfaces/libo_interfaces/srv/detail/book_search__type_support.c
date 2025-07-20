// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from libo_interfaces:srv/BookSearch.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "libo_interfaces/srv/detail/book_search__rosidl_typesupport_introspection_c.h"
#include "libo_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "libo_interfaces/srv/detail/book_search__functions.h"
#include "libo_interfaces/srv/detail/book_search__struct.h"


// Include directives for member types
// Member `query`
// Member `search_type`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void libo_interfaces__srv__BookSearch_Request__rosidl_typesupport_introspection_c__BookSearch_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  libo_interfaces__srv__BookSearch_Request__init(message_memory);
}

void libo_interfaces__srv__BookSearch_Request__rosidl_typesupport_introspection_c__BookSearch_Request_fini_function(void * message_memory)
{
  libo_interfaces__srv__BookSearch_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember libo_interfaces__srv__BookSearch_Request__rosidl_typesupport_introspection_c__BookSearch_Request_message_member_array[2] = {
  {
    "query",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(libo_interfaces__srv__BookSearch_Request, query),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "search_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(libo_interfaces__srv__BookSearch_Request, search_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers libo_interfaces__srv__BookSearch_Request__rosidl_typesupport_introspection_c__BookSearch_Request_message_members = {
  "libo_interfaces__srv",  // message namespace
  "BookSearch_Request",  // message name
  2,  // number of fields
  sizeof(libo_interfaces__srv__BookSearch_Request),
  false,  // has_any_key_member_
  libo_interfaces__srv__BookSearch_Request__rosidl_typesupport_introspection_c__BookSearch_Request_message_member_array,  // message members
  libo_interfaces__srv__BookSearch_Request__rosidl_typesupport_introspection_c__BookSearch_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  libo_interfaces__srv__BookSearch_Request__rosidl_typesupport_introspection_c__BookSearch_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t libo_interfaces__srv__BookSearch_Request__rosidl_typesupport_introspection_c__BookSearch_Request_message_type_support_handle = {
  0,
  &libo_interfaces__srv__BookSearch_Request__rosidl_typesupport_introspection_c__BookSearch_Request_message_members,
  get_message_typesupport_handle_function,
  &libo_interfaces__srv__BookSearch_Request__get_type_hash,
  &libo_interfaces__srv__BookSearch_Request__get_type_description,
  &libo_interfaces__srv__BookSearch_Request__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_libo_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, libo_interfaces, srv, BookSearch_Request)() {
  if (!libo_interfaces__srv__BookSearch_Request__rosidl_typesupport_introspection_c__BookSearch_Request_message_type_support_handle.typesupport_identifier) {
    libo_interfaces__srv__BookSearch_Request__rosidl_typesupport_introspection_c__BookSearch_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &libo_interfaces__srv__BookSearch_Request__rosidl_typesupport_introspection_c__BookSearch_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "libo_interfaces/srv/detail/book_search__rosidl_typesupport_introspection_c.h"
// already included above
// #include "libo_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "libo_interfaces/srv/detail/book_search__functions.h"
// already included above
// #include "libo_interfaces/srv/detail/book_search__struct.h"


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `books`
#include "libo_interfaces/msg/book_info.h"
// Member `books`
#include "libo_interfaces/msg/detail/book_info__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__BookSearch_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  libo_interfaces__srv__BookSearch_Response__init(message_memory);
}

void libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__BookSearch_Response_fini_function(void * message_memory)
{
  libo_interfaces__srv__BookSearch_Response__fini(message_memory);
}

size_t libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__size_function__BookSearch_Response__books(
  const void * untyped_member)
{
  const libo_interfaces__msg__BookInfo__Sequence * member =
    (const libo_interfaces__msg__BookInfo__Sequence *)(untyped_member);
  return member->size;
}

const void * libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__get_const_function__BookSearch_Response__books(
  const void * untyped_member, size_t index)
{
  const libo_interfaces__msg__BookInfo__Sequence * member =
    (const libo_interfaces__msg__BookInfo__Sequence *)(untyped_member);
  return &member->data[index];
}

void * libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__get_function__BookSearch_Response__books(
  void * untyped_member, size_t index)
{
  libo_interfaces__msg__BookInfo__Sequence * member =
    (libo_interfaces__msg__BookInfo__Sequence *)(untyped_member);
  return &member->data[index];
}

void libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__fetch_function__BookSearch_Response__books(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const libo_interfaces__msg__BookInfo * item =
    ((const libo_interfaces__msg__BookInfo *)
    libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__get_const_function__BookSearch_Response__books(untyped_member, index));
  libo_interfaces__msg__BookInfo * value =
    (libo_interfaces__msg__BookInfo *)(untyped_value);
  *value = *item;
}

void libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__assign_function__BookSearch_Response__books(
  void * untyped_member, size_t index, const void * untyped_value)
{
  libo_interfaces__msg__BookInfo * item =
    ((libo_interfaces__msg__BookInfo *)
    libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__get_function__BookSearch_Response__books(untyped_member, index));
  const libo_interfaces__msg__BookInfo * value =
    (const libo_interfaces__msg__BookInfo *)(untyped_value);
  *item = *value;
}

bool libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__resize_function__BookSearch_Response__books(
  void * untyped_member, size_t size)
{
  libo_interfaces__msg__BookInfo__Sequence * member =
    (libo_interfaces__msg__BookInfo__Sequence *)(untyped_member);
  libo_interfaces__msg__BookInfo__Sequence__fini(member);
  return libo_interfaces__msg__BookInfo__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__BookSearch_Response_message_member_array[3] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(libo_interfaces__srv__BookSearch_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(libo_interfaces__srv__BookSearch_Response, message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "books",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(libo_interfaces__srv__BookSearch_Response, books),  // bytes offset in struct
    NULL,  // default value
    libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__size_function__BookSearch_Response__books,  // size() function pointer
    libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__get_const_function__BookSearch_Response__books,  // get_const(index) function pointer
    libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__get_function__BookSearch_Response__books,  // get(index) function pointer
    libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__fetch_function__BookSearch_Response__books,  // fetch(index, &value) function pointer
    libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__assign_function__BookSearch_Response__books,  // assign(index, value) function pointer
    libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__resize_function__BookSearch_Response__books  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__BookSearch_Response_message_members = {
  "libo_interfaces__srv",  // message namespace
  "BookSearch_Response",  // message name
  3,  // number of fields
  sizeof(libo_interfaces__srv__BookSearch_Response),
  false,  // has_any_key_member_
  libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__BookSearch_Response_message_member_array,  // message members
  libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__BookSearch_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__BookSearch_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__BookSearch_Response_message_type_support_handle = {
  0,
  &libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__BookSearch_Response_message_members,
  get_message_typesupport_handle_function,
  &libo_interfaces__srv__BookSearch_Response__get_type_hash,
  &libo_interfaces__srv__BookSearch_Response__get_type_description,
  &libo_interfaces__srv__BookSearch_Response__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_libo_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, libo_interfaces, srv, BookSearch_Response)() {
  libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__BookSearch_Response_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, libo_interfaces, msg, BookInfo)();
  if (!libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__BookSearch_Response_message_type_support_handle.typesupport_identifier) {
    libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__BookSearch_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__BookSearch_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "libo_interfaces/srv/detail/book_search__rosidl_typesupport_introspection_c.h"
// already included above
// #include "libo_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "libo_interfaces/srv/detail/book_search__functions.h"
// already included above
// #include "libo_interfaces/srv/detail/book_search__struct.h"


// Include directives for member types
// Member `info`
#include "service_msgs/msg/service_event_info.h"
// Member `info`
#include "service_msgs/msg/detail/service_event_info__rosidl_typesupport_introspection_c.h"
// Member `request`
// Member `response`
#include "libo_interfaces/srv/book_search.h"
// Member `request`
// Member `response`
// already included above
// #include "libo_interfaces/srv/detail/book_search__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__BookSearch_Event_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  libo_interfaces__srv__BookSearch_Event__init(message_memory);
}

void libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__BookSearch_Event_fini_function(void * message_memory)
{
  libo_interfaces__srv__BookSearch_Event__fini(message_memory);
}

size_t libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__size_function__BookSearch_Event__request(
  const void * untyped_member)
{
  const libo_interfaces__srv__BookSearch_Request__Sequence * member =
    (const libo_interfaces__srv__BookSearch_Request__Sequence *)(untyped_member);
  return member->size;
}

const void * libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__get_const_function__BookSearch_Event__request(
  const void * untyped_member, size_t index)
{
  const libo_interfaces__srv__BookSearch_Request__Sequence * member =
    (const libo_interfaces__srv__BookSearch_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void * libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__get_function__BookSearch_Event__request(
  void * untyped_member, size_t index)
{
  libo_interfaces__srv__BookSearch_Request__Sequence * member =
    (libo_interfaces__srv__BookSearch_Request__Sequence *)(untyped_member);
  return &member->data[index];
}

void libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__fetch_function__BookSearch_Event__request(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const libo_interfaces__srv__BookSearch_Request * item =
    ((const libo_interfaces__srv__BookSearch_Request *)
    libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__get_const_function__BookSearch_Event__request(untyped_member, index));
  libo_interfaces__srv__BookSearch_Request * value =
    (libo_interfaces__srv__BookSearch_Request *)(untyped_value);
  *value = *item;
}

void libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__assign_function__BookSearch_Event__request(
  void * untyped_member, size_t index, const void * untyped_value)
{
  libo_interfaces__srv__BookSearch_Request * item =
    ((libo_interfaces__srv__BookSearch_Request *)
    libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__get_function__BookSearch_Event__request(untyped_member, index));
  const libo_interfaces__srv__BookSearch_Request * value =
    (const libo_interfaces__srv__BookSearch_Request *)(untyped_value);
  *item = *value;
}

bool libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__resize_function__BookSearch_Event__request(
  void * untyped_member, size_t size)
{
  libo_interfaces__srv__BookSearch_Request__Sequence * member =
    (libo_interfaces__srv__BookSearch_Request__Sequence *)(untyped_member);
  libo_interfaces__srv__BookSearch_Request__Sequence__fini(member);
  return libo_interfaces__srv__BookSearch_Request__Sequence__init(member, size);
}

size_t libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__size_function__BookSearch_Event__response(
  const void * untyped_member)
{
  const libo_interfaces__srv__BookSearch_Response__Sequence * member =
    (const libo_interfaces__srv__BookSearch_Response__Sequence *)(untyped_member);
  return member->size;
}

const void * libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__get_const_function__BookSearch_Event__response(
  const void * untyped_member, size_t index)
{
  const libo_interfaces__srv__BookSearch_Response__Sequence * member =
    (const libo_interfaces__srv__BookSearch_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void * libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__get_function__BookSearch_Event__response(
  void * untyped_member, size_t index)
{
  libo_interfaces__srv__BookSearch_Response__Sequence * member =
    (libo_interfaces__srv__BookSearch_Response__Sequence *)(untyped_member);
  return &member->data[index];
}

void libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__fetch_function__BookSearch_Event__response(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const libo_interfaces__srv__BookSearch_Response * item =
    ((const libo_interfaces__srv__BookSearch_Response *)
    libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__get_const_function__BookSearch_Event__response(untyped_member, index));
  libo_interfaces__srv__BookSearch_Response * value =
    (libo_interfaces__srv__BookSearch_Response *)(untyped_value);
  *value = *item;
}

void libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__assign_function__BookSearch_Event__response(
  void * untyped_member, size_t index, const void * untyped_value)
{
  libo_interfaces__srv__BookSearch_Response * item =
    ((libo_interfaces__srv__BookSearch_Response *)
    libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__get_function__BookSearch_Event__response(untyped_member, index));
  const libo_interfaces__srv__BookSearch_Response * value =
    (const libo_interfaces__srv__BookSearch_Response *)(untyped_value);
  *item = *value;
}

bool libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__resize_function__BookSearch_Event__response(
  void * untyped_member, size_t size)
{
  libo_interfaces__srv__BookSearch_Response__Sequence * member =
    (libo_interfaces__srv__BookSearch_Response__Sequence *)(untyped_member);
  libo_interfaces__srv__BookSearch_Response__Sequence__fini(member);
  return libo_interfaces__srv__BookSearch_Response__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__BookSearch_Event_message_member_array[3] = {
  {
    "info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(libo_interfaces__srv__BookSearch_Event, info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "request",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(libo_interfaces__srv__BookSearch_Event, request),  // bytes offset in struct
    NULL,  // default value
    libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__size_function__BookSearch_Event__request,  // size() function pointer
    libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__get_const_function__BookSearch_Event__request,  // get_const(index) function pointer
    libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__get_function__BookSearch_Event__request,  // get(index) function pointer
    libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__fetch_function__BookSearch_Event__request,  // fetch(index, &value) function pointer
    libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__assign_function__BookSearch_Event__request,  // assign(index, value) function pointer
    libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__resize_function__BookSearch_Event__request  // resize(index) function pointer
  },
  {
    "response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is key
    true,  // is array
    1,  // array size
    true,  // is upper bound
    offsetof(libo_interfaces__srv__BookSearch_Event, response),  // bytes offset in struct
    NULL,  // default value
    libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__size_function__BookSearch_Event__response,  // size() function pointer
    libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__get_const_function__BookSearch_Event__response,  // get_const(index) function pointer
    libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__get_function__BookSearch_Event__response,  // get(index) function pointer
    libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__fetch_function__BookSearch_Event__response,  // fetch(index, &value) function pointer
    libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__assign_function__BookSearch_Event__response,  // assign(index, value) function pointer
    libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__resize_function__BookSearch_Event__response  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__BookSearch_Event_message_members = {
  "libo_interfaces__srv",  // message namespace
  "BookSearch_Event",  // message name
  3,  // number of fields
  sizeof(libo_interfaces__srv__BookSearch_Event),
  false,  // has_any_key_member_
  libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__BookSearch_Event_message_member_array,  // message members
  libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__BookSearch_Event_init_function,  // function to initialize message memory (memory has to be allocated)
  libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__BookSearch_Event_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__BookSearch_Event_message_type_support_handle = {
  0,
  &libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__BookSearch_Event_message_members,
  get_message_typesupport_handle_function,
  &libo_interfaces__srv__BookSearch_Event__get_type_hash,
  &libo_interfaces__srv__BookSearch_Event__get_type_description,
  &libo_interfaces__srv__BookSearch_Event__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_libo_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, libo_interfaces, srv, BookSearch_Event)() {
  libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__BookSearch_Event_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, service_msgs, msg, ServiceEventInfo)();
  libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__BookSearch_Event_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, libo_interfaces, srv, BookSearch_Request)();
  libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__BookSearch_Event_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, libo_interfaces, srv, BookSearch_Response)();
  if (!libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__BookSearch_Event_message_type_support_handle.typesupport_identifier) {
    libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__BookSearch_Event_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__BookSearch_Event_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "libo_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "libo_interfaces/srv/detail/book_search__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers libo_interfaces__srv__detail__book_search__rosidl_typesupport_introspection_c__BookSearch_service_members = {
  "libo_interfaces__srv",  // service namespace
  "BookSearch",  // service name
  // the following fields are initialized below on first access
  NULL,  // request message
  // libo_interfaces__srv__detail__book_search__rosidl_typesupport_introspection_c__BookSearch_Request_message_type_support_handle,
  NULL,  // response message
  // libo_interfaces__srv__detail__book_search__rosidl_typesupport_introspection_c__BookSearch_Response_message_type_support_handle
  NULL  // event_message
  // libo_interfaces__srv__detail__book_search__rosidl_typesupport_introspection_c__BookSearch_Response_message_type_support_handle
};


static rosidl_service_type_support_t libo_interfaces__srv__detail__book_search__rosidl_typesupport_introspection_c__BookSearch_service_type_support_handle = {
  0,
  &libo_interfaces__srv__detail__book_search__rosidl_typesupport_introspection_c__BookSearch_service_members,
  get_service_typesupport_handle_function,
  &libo_interfaces__srv__BookSearch_Request__rosidl_typesupport_introspection_c__BookSearch_Request_message_type_support_handle,
  &libo_interfaces__srv__BookSearch_Response__rosidl_typesupport_introspection_c__BookSearch_Response_message_type_support_handle,
  &libo_interfaces__srv__BookSearch_Event__rosidl_typesupport_introspection_c__BookSearch_Event_message_type_support_handle,
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_CREATE_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    libo_interfaces,
    srv,
    BookSearch
  ),
  ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_DESTROY_EVENT_MESSAGE_SYMBOL_NAME(
    rosidl_typesupport_c,
    libo_interfaces,
    srv,
    BookSearch
  ),
  &libo_interfaces__srv__BookSearch__get_type_hash,
  &libo_interfaces__srv__BookSearch__get_type_description,
  &libo_interfaces__srv__BookSearch__get_type_description_sources,
};

// Forward declaration of message type support functions for service members
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, libo_interfaces, srv, BookSearch_Request)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, libo_interfaces, srv, BookSearch_Response)(void);

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, libo_interfaces, srv, BookSearch_Event)(void);

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_libo_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, libo_interfaces, srv, BookSearch)(void) {
  if (!libo_interfaces__srv__detail__book_search__rosidl_typesupport_introspection_c__BookSearch_service_type_support_handle.typesupport_identifier) {
    libo_interfaces__srv__detail__book_search__rosidl_typesupport_introspection_c__BookSearch_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)libo_interfaces__srv__detail__book_search__rosidl_typesupport_introspection_c__BookSearch_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, libo_interfaces, srv, BookSearch_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, libo_interfaces, srv, BookSearch_Response)()->data;
  }
  if (!service_members->event_members_) {
    service_members->event_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, libo_interfaces, srv, BookSearch_Event)()->data;
  }

  return &libo_interfaces__srv__detail__book_search__rosidl_typesupport_introspection_c__BookSearch_service_type_support_handle;
}
