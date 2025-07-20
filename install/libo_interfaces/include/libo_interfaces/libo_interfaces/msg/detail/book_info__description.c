// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from libo_interfaces:msg/BookInfo.idl
// generated code does not contain a copyright notice

#include "libo_interfaces/msg/detail/book_info__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_libo_interfaces
const rosidl_type_hash_t *
libo_interfaces__msg__BookInfo__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x18, 0x12, 0xf9, 0xb5, 0x49, 0x10, 0x55, 0xa6,
      0x7b, 0xe8, 0x5e, 0x51, 0x64, 0xd0, 0x74, 0x55,
      0x54, 0xef, 0xfe, 0x06, 0x25, 0x4e, 0x55, 0x6c,
      0x61, 0x6e, 0xee, 0x4d, 0xff, 0x17, 0xd0, 0xf5,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char libo_interfaces__msg__BookInfo__TYPE_NAME[] = "libo_interfaces/msg/BookInfo";

// Define type names, field names, and default values
static char libo_interfaces__msg__BookInfo__FIELD_NAME__id[] = "id";
static char libo_interfaces__msg__BookInfo__FIELD_NAME__title[] = "title";
static char libo_interfaces__msg__BookInfo__FIELD_NAME__author[] = "author";
static char libo_interfaces__msg__BookInfo__FIELD_NAME__publisher[] = "publisher";
static char libo_interfaces__msg__BookInfo__FIELD_NAME__category_name[] = "category_name";
static char libo_interfaces__msg__BookInfo__FIELD_NAME__location[] = "location";
static char libo_interfaces__msg__BookInfo__FIELD_NAME__price[] = "price";
static char libo_interfaces__msg__BookInfo__FIELD_NAME__stock_quantity[] = "stock_quantity";
static char libo_interfaces__msg__BookInfo__FIELD_NAME__isbn[] = "isbn";
static char libo_interfaces__msg__BookInfo__FIELD_NAME__cover_image_url[] = "cover_image_url";

static rosidl_runtime_c__type_description__Field libo_interfaces__msg__BookInfo__FIELDS[] = {
  {
    {libo_interfaces__msg__BookInfo__FIELD_NAME__id, 2, 2},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {libo_interfaces__msg__BookInfo__FIELD_NAME__title, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {libo_interfaces__msg__BookInfo__FIELD_NAME__author, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {libo_interfaces__msg__BookInfo__FIELD_NAME__publisher, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {libo_interfaces__msg__BookInfo__FIELD_NAME__category_name, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {libo_interfaces__msg__BookInfo__FIELD_NAME__location, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {libo_interfaces__msg__BookInfo__FIELD_NAME__price, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {libo_interfaces__msg__BookInfo__FIELD_NAME__stock_quantity, 14, 14},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_INT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {libo_interfaces__msg__BookInfo__FIELD_NAME__isbn, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {libo_interfaces__msg__BookInfo__FIELD_NAME__cover_image_url, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_STRING,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
libo_interfaces__msg__BookInfo__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {libo_interfaces__msg__BookInfo__TYPE_NAME, 28, 28},
      {libo_interfaces__msg__BookInfo__FIELDS, 10, 10},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "int32 id\n"
  "string title\n"
  "string author  \n"
  "string publisher\n"
  "string category_name\n"
  "string location\n"
  "float64 price\n"
  "int32 stock_quantity\n"
  "string isbn\n"
  "string cover_image_url";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
libo_interfaces__msg__BookInfo__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {libo_interfaces__msg__BookInfo__TYPE_NAME, 28, 28},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 161, 161},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
libo_interfaces__msg__BookInfo__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *libo_interfaces__msg__BookInfo__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
