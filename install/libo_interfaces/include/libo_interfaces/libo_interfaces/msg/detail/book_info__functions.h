// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from libo_interfaces:msg/BookInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "libo_interfaces/msg/book_info.h"


#ifndef LIBO_INTERFACES__MSG__DETAIL__BOOK_INFO__FUNCTIONS_H_
#define LIBO_INTERFACES__MSG__DETAIL__BOOK_INFO__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "libo_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "libo_interfaces/msg/detail/book_info__struct.h"

/// Initialize msg/BookInfo message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * libo_interfaces__msg__BookInfo
 * )) before or use
 * libo_interfaces__msg__BookInfo__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_libo_interfaces
bool
libo_interfaces__msg__BookInfo__init(libo_interfaces__msg__BookInfo * msg);

/// Finalize msg/BookInfo message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_libo_interfaces
void
libo_interfaces__msg__BookInfo__fini(libo_interfaces__msg__BookInfo * msg);

/// Create msg/BookInfo message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * libo_interfaces__msg__BookInfo__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_libo_interfaces
libo_interfaces__msg__BookInfo *
libo_interfaces__msg__BookInfo__create(void);

/// Destroy msg/BookInfo message.
/**
 * It calls
 * libo_interfaces__msg__BookInfo__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_libo_interfaces
void
libo_interfaces__msg__BookInfo__destroy(libo_interfaces__msg__BookInfo * msg);

/// Check for msg/BookInfo message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_libo_interfaces
bool
libo_interfaces__msg__BookInfo__are_equal(const libo_interfaces__msg__BookInfo * lhs, const libo_interfaces__msg__BookInfo * rhs);

/// Copy a msg/BookInfo message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_libo_interfaces
bool
libo_interfaces__msg__BookInfo__copy(
  const libo_interfaces__msg__BookInfo * input,
  libo_interfaces__msg__BookInfo * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_libo_interfaces
const rosidl_type_hash_t *
libo_interfaces__msg__BookInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_libo_interfaces
const rosidl_runtime_c__type_description__TypeDescription *
libo_interfaces__msg__BookInfo__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_libo_interfaces
const rosidl_runtime_c__type_description__TypeSource *
libo_interfaces__msg__BookInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_libo_interfaces
const rosidl_runtime_c__type_description__TypeSource__Sequence *
libo_interfaces__msg__BookInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/BookInfo messages.
/**
 * It allocates the memory for the number of elements and calls
 * libo_interfaces__msg__BookInfo__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_libo_interfaces
bool
libo_interfaces__msg__BookInfo__Sequence__init(libo_interfaces__msg__BookInfo__Sequence * array, size_t size);

/// Finalize array of msg/BookInfo messages.
/**
 * It calls
 * libo_interfaces__msg__BookInfo__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_libo_interfaces
void
libo_interfaces__msg__BookInfo__Sequence__fini(libo_interfaces__msg__BookInfo__Sequence * array);

/// Create array of msg/BookInfo messages.
/**
 * It allocates the memory for the array and calls
 * libo_interfaces__msg__BookInfo__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_libo_interfaces
libo_interfaces__msg__BookInfo__Sequence *
libo_interfaces__msg__BookInfo__Sequence__create(size_t size);

/// Destroy array of msg/BookInfo messages.
/**
 * It calls
 * libo_interfaces__msg__BookInfo__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_libo_interfaces
void
libo_interfaces__msg__BookInfo__Sequence__destroy(libo_interfaces__msg__BookInfo__Sequence * array);

/// Check for msg/BookInfo message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_libo_interfaces
bool
libo_interfaces__msg__BookInfo__Sequence__are_equal(const libo_interfaces__msg__BookInfo__Sequence * lhs, const libo_interfaces__msg__BookInfo__Sequence * rhs);

/// Copy an array of msg/BookInfo messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_libo_interfaces
bool
libo_interfaces__msg__BookInfo__Sequence__copy(
  const libo_interfaces__msg__BookInfo__Sequence * input,
  libo_interfaces__msg__BookInfo__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // LIBO_INTERFACES__MSG__DETAIL__BOOK_INFO__FUNCTIONS_H_
