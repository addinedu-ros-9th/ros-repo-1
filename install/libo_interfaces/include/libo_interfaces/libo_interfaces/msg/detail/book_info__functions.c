// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from libo_interfaces:msg/BookInfo.idl
// generated code does not contain a copyright notice
#include "libo_interfaces/msg/detail/book_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `title`
// Member `author`
// Member `publisher`
// Member `category_name`
// Member `location`
// Member `isbn`
// Member `cover_image_url`
#include "rosidl_runtime_c/string_functions.h"

bool
libo_interfaces__msg__BookInfo__init(libo_interfaces__msg__BookInfo * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // title
  if (!rosidl_runtime_c__String__init(&msg->title)) {
    libo_interfaces__msg__BookInfo__fini(msg);
    return false;
  }
  // author
  if (!rosidl_runtime_c__String__init(&msg->author)) {
    libo_interfaces__msg__BookInfo__fini(msg);
    return false;
  }
  // publisher
  if (!rosidl_runtime_c__String__init(&msg->publisher)) {
    libo_interfaces__msg__BookInfo__fini(msg);
    return false;
  }
  // category_name
  if (!rosidl_runtime_c__String__init(&msg->category_name)) {
    libo_interfaces__msg__BookInfo__fini(msg);
    return false;
  }
  // location
  if (!rosidl_runtime_c__String__init(&msg->location)) {
    libo_interfaces__msg__BookInfo__fini(msg);
    return false;
  }
  // price
  // stock_quantity
  // isbn
  if (!rosidl_runtime_c__String__init(&msg->isbn)) {
    libo_interfaces__msg__BookInfo__fini(msg);
    return false;
  }
  // cover_image_url
  if (!rosidl_runtime_c__String__init(&msg->cover_image_url)) {
    libo_interfaces__msg__BookInfo__fini(msg);
    return false;
  }
  return true;
}

void
libo_interfaces__msg__BookInfo__fini(libo_interfaces__msg__BookInfo * msg)
{
  if (!msg) {
    return;
  }
  // id
  // title
  rosidl_runtime_c__String__fini(&msg->title);
  // author
  rosidl_runtime_c__String__fini(&msg->author);
  // publisher
  rosidl_runtime_c__String__fini(&msg->publisher);
  // category_name
  rosidl_runtime_c__String__fini(&msg->category_name);
  // location
  rosidl_runtime_c__String__fini(&msg->location);
  // price
  // stock_quantity
  // isbn
  rosidl_runtime_c__String__fini(&msg->isbn);
  // cover_image_url
  rosidl_runtime_c__String__fini(&msg->cover_image_url);
}

bool
libo_interfaces__msg__BookInfo__are_equal(const libo_interfaces__msg__BookInfo * lhs, const libo_interfaces__msg__BookInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // title
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->title), &(rhs->title)))
  {
    return false;
  }
  // author
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->author), &(rhs->author)))
  {
    return false;
  }
  // publisher
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->publisher), &(rhs->publisher)))
  {
    return false;
  }
  // category_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->category_name), &(rhs->category_name)))
  {
    return false;
  }
  // location
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->location), &(rhs->location)))
  {
    return false;
  }
  // price
  if (lhs->price != rhs->price) {
    return false;
  }
  // stock_quantity
  if (lhs->stock_quantity != rhs->stock_quantity) {
    return false;
  }
  // isbn
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->isbn), &(rhs->isbn)))
  {
    return false;
  }
  // cover_image_url
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->cover_image_url), &(rhs->cover_image_url)))
  {
    return false;
  }
  return true;
}

bool
libo_interfaces__msg__BookInfo__copy(
  const libo_interfaces__msg__BookInfo * input,
  libo_interfaces__msg__BookInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // title
  if (!rosidl_runtime_c__String__copy(
      &(input->title), &(output->title)))
  {
    return false;
  }
  // author
  if (!rosidl_runtime_c__String__copy(
      &(input->author), &(output->author)))
  {
    return false;
  }
  // publisher
  if (!rosidl_runtime_c__String__copy(
      &(input->publisher), &(output->publisher)))
  {
    return false;
  }
  // category_name
  if (!rosidl_runtime_c__String__copy(
      &(input->category_name), &(output->category_name)))
  {
    return false;
  }
  // location
  if (!rosidl_runtime_c__String__copy(
      &(input->location), &(output->location)))
  {
    return false;
  }
  // price
  output->price = input->price;
  // stock_quantity
  output->stock_quantity = input->stock_quantity;
  // isbn
  if (!rosidl_runtime_c__String__copy(
      &(input->isbn), &(output->isbn)))
  {
    return false;
  }
  // cover_image_url
  if (!rosidl_runtime_c__String__copy(
      &(input->cover_image_url), &(output->cover_image_url)))
  {
    return false;
  }
  return true;
}

libo_interfaces__msg__BookInfo *
libo_interfaces__msg__BookInfo__create(void)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  libo_interfaces__msg__BookInfo * msg = (libo_interfaces__msg__BookInfo *)allocator.allocate(sizeof(libo_interfaces__msg__BookInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(libo_interfaces__msg__BookInfo));
  bool success = libo_interfaces__msg__BookInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
libo_interfaces__msg__BookInfo__destroy(libo_interfaces__msg__BookInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    libo_interfaces__msg__BookInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
libo_interfaces__msg__BookInfo__Sequence__init(libo_interfaces__msg__BookInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  libo_interfaces__msg__BookInfo * data = NULL;

  if (size) {
    data = (libo_interfaces__msg__BookInfo *)allocator.zero_allocate(size, sizeof(libo_interfaces__msg__BookInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = libo_interfaces__msg__BookInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        libo_interfaces__msg__BookInfo__fini(&data[i - 1]);
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
libo_interfaces__msg__BookInfo__Sequence__fini(libo_interfaces__msg__BookInfo__Sequence * array)
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
      libo_interfaces__msg__BookInfo__fini(&array->data[i]);
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

libo_interfaces__msg__BookInfo__Sequence *
libo_interfaces__msg__BookInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  libo_interfaces__msg__BookInfo__Sequence * array = (libo_interfaces__msg__BookInfo__Sequence *)allocator.allocate(sizeof(libo_interfaces__msg__BookInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = libo_interfaces__msg__BookInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
libo_interfaces__msg__BookInfo__Sequence__destroy(libo_interfaces__msg__BookInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    libo_interfaces__msg__BookInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
libo_interfaces__msg__BookInfo__Sequence__are_equal(const libo_interfaces__msg__BookInfo__Sequence * lhs, const libo_interfaces__msg__BookInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!libo_interfaces__msg__BookInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
libo_interfaces__msg__BookInfo__Sequence__copy(
  const libo_interfaces__msg__BookInfo__Sequence * input,
  libo_interfaces__msg__BookInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(libo_interfaces__msg__BookInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    libo_interfaces__msg__BookInfo * data =
      (libo_interfaces__msg__BookInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!libo_interfaces__msg__BookInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          libo_interfaces__msg__BookInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!libo_interfaces__msg__BookInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
