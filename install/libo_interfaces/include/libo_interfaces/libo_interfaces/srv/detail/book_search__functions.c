// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from libo_interfaces:srv/BookSearch.idl
// generated code does not contain a copyright notice
#include "libo_interfaces/srv/detail/book_search__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `query`
// Member `search_type`
#include "rosidl_runtime_c/string_functions.h"

bool
libo_interfaces__srv__BookSearch_Request__init(libo_interfaces__srv__BookSearch_Request * msg)
{
  if (!msg) {
    return false;
  }
  // query
  if (!rosidl_runtime_c__String__init(&msg->query)) {
    libo_interfaces__srv__BookSearch_Request__fini(msg);
    return false;
  }
  // search_type
  if (!rosidl_runtime_c__String__init(&msg->search_type)) {
    libo_interfaces__srv__BookSearch_Request__fini(msg);
    return false;
  }
  return true;
}

void
libo_interfaces__srv__BookSearch_Request__fini(libo_interfaces__srv__BookSearch_Request * msg)
{
  if (!msg) {
    return;
  }
  // query
  rosidl_runtime_c__String__fini(&msg->query);
  // search_type
  rosidl_runtime_c__String__fini(&msg->search_type);
}

bool
libo_interfaces__srv__BookSearch_Request__are_equal(const libo_interfaces__srv__BookSearch_Request * lhs, const libo_interfaces__srv__BookSearch_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // query
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->query), &(rhs->query)))
  {
    return false;
  }
  // search_type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->search_type), &(rhs->search_type)))
  {
    return false;
  }
  return true;
}

bool
libo_interfaces__srv__BookSearch_Request__copy(
  const libo_interfaces__srv__BookSearch_Request * input,
  libo_interfaces__srv__BookSearch_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // query
  if (!rosidl_runtime_c__String__copy(
      &(input->query), &(output->query)))
  {
    return false;
  }
  // search_type
  if (!rosidl_runtime_c__String__copy(
      &(input->search_type), &(output->search_type)))
  {
    return false;
  }
  return true;
}

libo_interfaces__srv__BookSearch_Request *
libo_interfaces__srv__BookSearch_Request__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  libo_interfaces__srv__BookSearch_Request * msg = (libo_interfaces__srv__BookSearch_Request *)allocator.allocate(sizeof(libo_interfaces__srv__BookSearch_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(libo_interfaces__srv__BookSearch_Request));
  bool success = libo_interfaces__srv__BookSearch_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
libo_interfaces__srv__BookSearch_Request__destroy(libo_interfaces__srv__BookSearch_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    libo_interfaces__srv__BookSearch_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
libo_interfaces__srv__BookSearch_Request__Sequence__init(libo_interfaces__srv__BookSearch_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  libo_interfaces__srv__BookSearch_Request * data = NULL;

  if (size) {
    data = (libo_interfaces__srv__BookSearch_Request *)allocator.zero_allocate(size, sizeof(libo_interfaces__srv__BookSearch_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = libo_interfaces__srv__BookSearch_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        libo_interfaces__srv__BookSearch_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
libo_interfaces__srv__BookSearch_Request__Sequence__fini(libo_interfaces__srv__BookSearch_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      libo_interfaces__srv__BookSearch_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

libo_interfaces__srv__BookSearch_Request__Sequence *
libo_interfaces__srv__BookSearch_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  libo_interfaces__srv__BookSearch_Request__Sequence * array = (libo_interfaces__srv__BookSearch_Request__Sequence *)allocator.allocate(sizeof(libo_interfaces__srv__BookSearch_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = libo_interfaces__srv__BookSearch_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
libo_interfaces__srv__BookSearch_Request__Sequence__destroy(libo_interfaces__srv__BookSearch_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    libo_interfaces__srv__BookSearch_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
libo_interfaces__srv__BookSearch_Request__Sequence__are_equal(const libo_interfaces__srv__BookSearch_Request__Sequence * lhs, const libo_interfaces__srv__BookSearch_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!libo_interfaces__srv__BookSearch_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
libo_interfaces__srv__BookSearch_Request__Sequence__copy(
  const libo_interfaces__srv__BookSearch_Request__Sequence * input,
  libo_interfaces__srv__BookSearch_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(libo_interfaces__srv__BookSearch_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    libo_interfaces__srv__BookSearch_Request * data =
      (libo_interfaces__srv__BookSearch_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!libo_interfaces__srv__BookSearch_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          libo_interfaces__srv__BookSearch_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!libo_interfaces__srv__BookSearch_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"
// Member `books`
#include "libo_interfaces/msg/detail/book_info__functions.h"

bool
libo_interfaces__srv__BookSearch_Response__init(libo_interfaces__srv__BookSearch_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    libo_interfaces__srv__BookSearch_Response__fini(msg);
    return false;
  }
  // books
  if (!libo_interfaces__msg__BookInfo__Sequence__init(&msg->books, 0)) {
    libo_interfaces__srv__BookSearch_Response__fini(msg);
    return false;
  }
  return true;
}

void
libo_interfaces__srv__BookSearch_Response__fini(libo_interfaces__srv__BookSearch_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
  // books
  libo_interfaces__msg__BookInfo__Sequence__fini(&msg->books);
}

bool
libo_interfaces__srv__BookSearch_Response__are_equal(const libo_interfaces__srv__BookSearch_Response * lhs, const libo_interfaces__srv__BookSearch_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  // books
  if (!libo_interfaces__msg__BookInfo__Sequence__are_equal(
      &(lhs->books), &(rhs->books)))
  {
    return false;
  }
  return true;
}

bool
libo_interfaces__srv__BookSearch_Response__copy(
  const libo_interfaces__srv__BookSearch_Response * input,
  libo_interfaces__srv__BookSearch_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  // books
  if (!libo_interfaces__msg__BookInfo__Sequence__copy(
      &(input->books), &(output->books)))
  {
    return false;
  }
  return true;
}

libo_interfaces__srv__BookSearch_Response *
libo_interfaces__srv__BookSearch_Response__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  libo_interfaces__srv__BookSearch_Response * msg = (libo_interfaces__srv__BookSearch_Response *)allocator.allocate(sizeof(libo_interfaces__srv__BookSearch_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(libo_interfaces__srv__BookSearch_Response));
  bool success = libo_interfaces__srv__BookSearch_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
libo_interfaces__srv__BookSearch_Response__destroy(libo_interfaces__srv__BookSearch_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    libo_interfaces__srv__BookSearch_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
libo_interfaces__srv__BookSearch_Response__Sequence__init(libo_interfaces__srv__BookSearch_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  libo_interfaces__srv__BookSearch_Response * data = NULL;

  if (size) {
    data = (libo_interfaces__srv__BookSearch_Response *)allocator.zero_allocate(size, sizeof(libo_interfaces__srv__BookSearch_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = libo_interfaces__srv__BookSearch_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        libo_interfaces__srv__BookSearch_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
libo_interfaces__srv__BookSearch_Response__Sequence__fini(libo_interfaces__srv__BookSearch_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      libo_interfaces__srv__BookSearch_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

libo_interfaces__srv__BookSearch_Response__Sequence *
libo_interfaces__srv__BookSearch_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  libo_interfaces__srv__BookSearch_Response__Sequence * array = (libo_interfaces__srv__BookSearch_Response__Sequence *)allocator.allocate(sizeof(libo_interfaces__srv__BookSearch_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = libo_interfaces__srv__BookSearch_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
libo_interfaces__srv__BookSearch_Response__Sequence__destroy(libo_interfaces__srv__BookSearch_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    libo_interfaces__srv__BookSearch_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
libo_interfaces__srv__BookSearch_Response__Sequence__are_equal(const libo_interfaces__srv__BookSearch_Response__Sequence * lhs, const libo_interfaces__srv__BookSearch_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!libo_interfaces__srv__BookSearch_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
libo_interfaces__srv__BookSearch_Response__Sequence__copy(
  const libo_interfaces__srv__BookSearch_Response__Sequence * input,
  libo_interfaces__srv__BookSearch_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(libo_interfaces__srv__BookSearch_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    libo_interfaces__srv__BookSearch_Response * data =
      (libo_interfaces__srv__BookSearch_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!libo_interfaces__srv__BookSearch_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          libo_interfaces__srv__BookSearch_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!libo_interfaces__srv__BookSearch_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `info`
#include "service_msgs/msg/detail/service_event_info__functions.h"
// Member `request`
// Member `response`
// already included above
// #include "libo_interfaces/srv/detail/book_search__functions.h"

bool
libo_interfaces__srv__BookSearch_Event__init(libo_interfaces__srv__BookSearch_Event * msg)
{
  if (!msg) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__init(&msg->info)) {
    libo_interfaces__srv__BookSearch_Event__fini(msg);
    return false;
  }
  // request
  if (!libo_interfaces__srv__BookSearch_Request__Sequence__init(&msg->request, 0)) {
    libo_interfaces__srv__BookSearch_Event__fini(msg);
    return false;
  }
  // response
  if (!libo_interfaces__srv__BookSearch_Response__Sequence__init(&msg->response, 0)) {
    libo_interfaces__srv__BookSearch_Event__fini(msg);
    return false;
  }
  return true;
}

void
libo_interfaces__srv__BookSearch_Event__fini(libo_interfaces__srv__BookSearch_Event * msg)
{
  if (!msg) {
    return;
  }
  // info
  service_msgs__msg__ServiceEventInfo__fini(&msg->info);
  // request
  libo_interfaces__srv__BookSearch_Request__Sequence__fini(&msg->request);
  // response
  libo_interfaces__srv__BookSearch_Response__Sequence__fini(&msg->response);
}

bool
libo_interfaces__srv__BookSearch_Event__are_equal(const libo_interfaces__srv__BookSearch_Event * lhs, const libo_interfaces__srv__BookSearch_Event * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__are_equal(
      &(lhs->info), &(rhs->info)))
  {
    return false;
  }
  // request
  if (!libo_interfaces__srv__BookSearch_Request__Sequence__are_equal(
      &(lhs->request), &(rhs->request)))
  {
    return false;
  }
  // response
  if (!libo_interfaces__srv__BookSearch_Response__Sequence__are_equal(
      &(lhs->response), &(rhs->response)))
  {
    return false;
  }
  return true;
}

bool
libo_interfaces__srv__BookSearch_Event__copy(
  const libo_interfaces__srv__BookSearch_Event * input,
  libo_interfaces__srv__BookSearch_Event * output)
{
  if (!input || !output) {
    return false;
  }
  // info
  if (!service_msgs__msg__ServiceEventInfo__copy(
      &(input->info), &(output->info)))
  {
    return false;
  }
  // request
  if (!libo_interfaces__srv__BookSearch_Request__Sequence__copy(
      &(input->request), &(output->request)))
  {
    return false;
  }
  // response
  if (!libo_interfaces__srv__BookSearch_Response__Sequence__copy(
      &(input->response), &(output->response)))
  {
    return false;
  }
  return true;
}

libo_interfaces__srv__BookSearch_Event *
libo_interfaces__srv__BookSearch_Event__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  libo_interfaces__srv__BookSearch_Event * msg = (libo_interfaces__srv__BookSearch_Event *)allocator.allocate(sizeof(libo_interfaces__srv__BookSearch_Event), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(libo_interfaces__srv__BookSearch_Event));
  bool success = libo_interfaces__srv__BookSearch_Event__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
libo_interfaces__srv__BookSearch_Event__destroy(libo_interfaces__srv__BookSearch_Event * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    libo_interfaces__srv__BookSearch_Event__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
libo_interfaces__srv__BookSearch_Event__Sequence__init(libo_interfaces__srv__BookSearch_Event__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  libo_interfaces__srv__BookSearch_Event * data = NULL;

  if (size) {
    data = (libo_interfaces__srv__BookSearch_Event *)allocator.zero_allocate(size, sizeof(libo_interfaces__srv__BookSearch_Event), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = libo_interfaces__srv__BookSearch_Event__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        libo_interfaces__srv__BookSearch_Event__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
libo_interfaces__srv__BookSearch_Event__Sequence__fini(libo_interfaces__srv__BookSearch_Event__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      libo_interfaces__srv__BookSearch_Event__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

libo_interfaces__srv__BookSearch_Event__Sequence *
libo_interfaces__srv__BookSearch_Event__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  libo_interfaces__srv__BookSearch_Event__Sequence * array = (libo_interfaces__srv__BookSearch_Event__Sequence *)allocator.allocate(sizeof(libo_interfaces__srv__BookSearch_Event__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = libo_interfaces__srv__BookSearch_Event__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
libo_interfaces__srv__BookSearch_Event__Sequence__destroy(libo_interfaces__srv__BookSearch_Event__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    libo_interfaces__srv__BookSearch_Event__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
libo_interfaces__srv__BookSearch_Event__Sequence__are_equal(const libo_interfaces__srv__BookSearch_Event__Sequence * lhs, const libo_interfaces__srv__BookSearch_Event__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!libo_interfaces__srv__BookSearch_Event__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
libo_interfaces__srv__BookSearch_Event__Sequence__copy(
  const libo_interfaces__srv__BookSearch_Event__Sequence * input,
  libo_interfaces__srv__BookSearch_Event__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(libo_interfaces__srv__BookSearch_Event);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    libo_interfaces__srv__BookSearch_Event * data =
      (libo_interfaces__srv__BookSearch_Event *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!libo_interfaces__srv__BookSearch_Event__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          libo_interfaces__srv__BookSearch_Event__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!libo_interfaces__srv__BookSearch_Event__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
