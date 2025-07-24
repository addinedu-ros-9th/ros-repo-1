// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from libo_interfaces:srv/Navigate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "libo_interfaces/srv/navigate.h"


#ifndef LIBO_INTERFACES__SRV__DETAIL__NAVIGATE__STRUCT_H_
#define LIBO_INTERFACES__SRV__DETAIL__NAVIGATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Navigate in the package libo_interfaces.
typedef struct libo_interfaces__srv__Navigate_Request
{
  /// 목표 x 좌표
  float x;
  /// 목표 y 좌표
  float y;
  /// 목표 yaw 각도 (방향)
  float yaw;
} libo_interfaces__srv__Navigate_Request;

// Struct for a sequence of libo_interfaces__srv__Navigate_Request.
typedef struct libo_interfaces__srv__Navigate_Request__Sequence
{
  libo_interfaces__srv__Navigate_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} libo_interfaces__srv__Navigate_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/Navigate in the package libo_interfaces.
typedef struct libo_interfaces__srv__Navigate_Response
{
  /// 성공 여부
  bool success;
  /// 결과 메시지
  rosidl_runtime_c__String message;
} libo_interfaces__srv__Navigate_Response;

// Struct for a sequence of libo_interfaces__srv__Navigate_Response.
typedef struct libo_interfaces__srv__Navigate_Response__Sequence
{
  libo_interfaces__srv__Navigate_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} libo_interfaces__srv__Navigate_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  libo_interfaces__srv__Navigate_Event__request__MAX_SIZE = 1
};
// response
enum
{
  libo_interfaces__srv__Navigate_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/Navigate in the package libo_interfaces.
typedef struct libo_interfaces__srv__Navigate_Event
{
  service_msgs__msg__ServiceEventInfo info;
  libo_interfaces__srv__Navigate_Request__Sequence request;
  libo_interfaces__srv__Navigate_Response__Sequence response;
} libo_interfaces__srv__Navigate_Event;

// Struct for a sequence of libo_interfaces__srv__Navigate_Event.
typedef struct libo_interfaces__srv__Navigate_Event__Sequence
{
  libo_interfaces__srv__Navigate_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} libo_interfaces__srv__Navigate_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIBO_INTERFACES__SRV__DETAIL__NAVIGATE__STRUCT_H_
