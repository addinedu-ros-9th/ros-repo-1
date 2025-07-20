// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from libo_interfaces:msg/BookInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "libo_interfaces/msg/book_info.h"


#ifndef LIBO_INTERFACES__MSG__DETAIL__BOOK_INFO__STRUCT_H_
#define LIBO_INTERFACES__MSG__DETAIL__BOOK_INFO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'title'
// Member 'author'
// Member 'publisher'
// Member 'category_name'
// Member 'location'
// Member 'isbn'
// Member 'cover_image_url'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/BookInfo in the package libo_interfaces.
typedef struct libo_interfaces__msg__BookInfo
{
  int32_t id;
  rosidl_runtime_c__String title;
  rosidl_runtime_c__String author;
  rosidl_runtime_c__String publisher;
  rosidl_runtime_c__String category_name;
  rosidl_runtime_c__String location;
  double price;
  int32_t stock_quantity;
  rosidl_runtime_c__String isbn;
  rosidl_runtime_c__String cover_image_url;
} libo_interfaces__msg__BookInfo;

// Struct for a sequence of libo_interfaces__msg__BookInfo.
typedef struct libo_interfaces__msg__BookInfo__Sequence
{
  libo_interfaces__msg__BookInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} libo_interfaces__msg__BookInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIBO_INTERFACES__MSG__DETAIL__BOOK_INFO__STRUCT_H_
