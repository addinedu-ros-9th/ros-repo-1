// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from libo_interfaces:srv/BookSearch.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "libo_interfaces/srv/book_search.h"


#ifndef LIBO_INTERFACES__SRV__DETAIL__BOOK_SEARCH__STRUCT_H_
#define LIBO_INTERFACES__SRV__DETAIL__BOOK_SEARCH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'query'
// Member 'search_type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/BookSearch in the package libo_interfaces.
typedef struct libo_interfaces__srv__BookSearch_Request
{
  /// 검색어
  rosidl_runtime_c__String query;
  /// 검색 타입: "title", "author", "publisher"
  rosidl_runtime_c__String search_type;
} libo_interfaces__srv__BookSearch_Request;

// Struct for a sequence of libo_interfaces__srv__BookSearch_Request.
typedef struct libo_interfaces__srv__BookSearch_Request__Sequence
{
  libo_interfaces__srv__BookSearch_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} libo_interfaces__srv__BookSearch_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"
// Member 'books'
#include "libo_interfaces/msg/detail/book_info__struct.h"

/// Struct defined in srv/BookSearch in the package libo_interfaces.
typedef struct libo_interfaces__srv__BookSearch_Response
{
  /// 검색 성공 여부
  bool success;
  /// 오류 메시지 (실패 시)
  rosidl_runtime_c__String message;
  /// 검색된 도서 리스트
  libo_interfaces__msg__BookInfo__Sequence books;
} libo_interfaces__srv__BookSearch_Response;

// Struct for a sequence of libo_interfaces__srv__BookSearch_Response.
typedef struct libo_interfaces__srv__BookSearch_Response__Sequence
{
  libo_interfaces__srv__BookSearch_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} libo_interfaces__srv__BookSearch_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  libo_interfaces__srv__BookSearch_Event__request__MAX_SIZE = 1
};
// response
enum
{
  libo_interfaces__srv__BookSearch_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/BookSearch in the package libo_interfaces.
typedef struct libo_interfaces__srv__BookSearch_Event
{
  service_msgs__msg__ServiceEventInfo info;
  libo_interfaces__srv__BookSearch_Request__Sequence request;
  libo_interfaces__srv__BookSearch_Response__Sequence response;
} libo_interfaces__srv__BookSearch_Event;

// Struct for a sequence of libo_interfaces__srv__BookSearch_Event.
typedef struct libo_interfaces__srv__BookSearch_Event__Sequence
{
  libo_interfaces__srv__BookSearch_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} libo_interfaces__srv__BookSearch_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIBO_INTERFACES__SRV__DETAIL__BOOK_SEARCH__STRUCT_H_
