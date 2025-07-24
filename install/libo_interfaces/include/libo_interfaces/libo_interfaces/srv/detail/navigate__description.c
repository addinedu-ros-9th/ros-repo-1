// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from libo_interfaces:srv/Navigate.idl
// generated code does not contain a copyright notice

#include "libo_interfaces/srv/detail/navigate__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_libo_interfaces
const rosidl_type_hash_t *
libo_interfaces__srv__Navigate__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x91, 0x0e, 0xba, 0x9a, 0xa0, 0xe5, 0xbf, 0x95,
      0xa2, 0xc4, 0x2a, 0xef, 0xbc, 0xce, 0xbe, 0x25,
      0x07, 0x4e, 0x3f, 0xfa, 0xb7, 0x92, 0xdc, 0x6d,
      0x53, 0xa2, 0xb7, 0x2b, 0x63, 0x70, 0x97, 0x27,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_libo_interfaces
const rosidl_type_hash_t *
libo_interfaces__srv__Navigate_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x56, 0x9c, 0x84, 0x36, 0xa7, 0x61, 0xc6, 0xdf,
      0x95, 0xce, 0x29, 0xa2, 0xe4, 0xdd, 0x44, 0x3d,
      0x74, 0xc9, 0x27, 0x32, 0xa6, 0x90, 0x55, 0x56,
      0x03, 0x54, 0x15, 0x6a, 0xd5, 0xa4, 0xa5, 0x23,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_libo_interfaces
const rosidl_type_hash_t *
libo_interfaces__srv__Navigate_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x67, 0xe3, 0x5e, 0x38, 0x3d, 0x84, 0xbf, 0x1b,
      0x0f, 0xac, 0x76, 0x3b, 0xab, 0x66, 0xec, 0xb0,
      0xc6, 0x26, 0x83, 0x72, 0x80, 0x15, 0xe9, 0xdd,
      0x09, 0x95, 0xd1, 0xc3, 0x53, 0xee, 0x7b, 0xf8,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_libo_interfaces
const rosidl_type_hash_t *
libo_interfaces__srv__Navigate_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x9a, 0xf5, 0x36, 0x8a, 0x95, 0x07, 0x1b, 0x88,
      0x10, 0x6f, 0x54, 0x8d, 0x61, 0x5f, 0xbe, 0xa1,
      0x4f, 0x2d, 0x81, 0xfd, 0x05, 0xda, 0x79, 0xe2,
      0x9b, 0x61, 0x6f, 0x37, 0x98, 0xe2, 0x50, 0x9d,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "builtin_interfaces/msg/detail/time__functions.h"
#include "service_msgs/msg/detail/service_event_info__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char libo_interfaces__srv__Navigate__TYPE_NAME[] = "libo_interfaces/srv/Navigate";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char libo_interfaces__srv__Navigate_Event__TYPE_NAME[] = "libo_interfaces/srv/Navigate_Event";
static char libo_interfaces__srv__Navigate_Request__TYPE_NAME[] = "libo_interfaces/srv/Navigate_Request";
static char libo_interfaces__srv__Navigate_Response__TYPE_NAME[] = "libo_interfaces/srv/Navigate_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char libo_interfaces__srv__Navigate__FIELD_NAME__request_message[] = "request_message";
static char libo_interfaces__srv__Navigate__FIELD_NAME__response_message[] = "response_message";
static char libo_interfaces__srv__Navigate__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field libo_interfaces__srv__Navigate__FIELDS[] = {
  {
    {libo_interfaces__srv__Navigate__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {libo_interfaces__srv__Navigate_Request__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
  {
    {libo_interfaces__srv__Navigate__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {libo_interfaces__srv__Navigate_Response__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
  {
    {libo_interfaces__srv__Navigate__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {libo_interfaces__srv__Navigate_Event__TYPE_NAME, 34, 34},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription libo_interfaces__srv__Navigate__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {libo_interfaces__srv__Navigate_Event__TYPE_NAME, 34, 34},
    {NULL, 0, 0},
  },
  {
    {libo_interfaces__srv__Navigate_Request__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {libo_interfaces__srv__Navigate_Response__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
libo_interfaces__srv__Navigate__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {libo_interfaces__srv__Navigate__TYPE_NAME, 28, 28},
      {libo_interfaces__srv__Navigate__FIELDS, 3, 3},
    },
    {libo_interfaces__srv__Navigate__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = libo_interfaces__srv__Navigate_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = libo_interfaces__srv__Navigate_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = libo_interfaces__srv__Navigate_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char libo_interfaces__srv__Navigate_Request__FIELD_NAME__x[] = "x";
static char libo_interfaces__srv__Navigate_Request__FIELD_NAME__y[] = "y";
static char libo_interfaces__srv__Navigate_Request__FIELD_NAME__yaw[] = "yaw";

static rosidl_runtime_c__type_description__Field libo_interfaces__srv__Navigate_Request__FIELDS[] = {
  {
    {libo_interfaces__srv__Navigate_Request__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {libo_interfaces__srv__Navigate_Request__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {libo_interfaces__srv__Navigate_Request__FIELD_NAME__yaw, 3, 3},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
libo_interfaces__srv__Navigate_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {libo_interfaces__srv__Navigate_Request__TYPE_NAME, 36, 36},
      {libo_interfaces__srv__Navigate_Request__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char libo_interfaces__srv__Navigate_Response__FIELD_NAME__success[] = "success";
static char libo_interfaces__srv__Navigate_Response__FIELD_NAME__message[] = "message";

static rosidl_runtime_c__type_description__Field libo_interfaces__srv__Navigate_Response__FIELDS[] = {
  {
    {libo_interfaces__srv__Navigate_Response__FIELD_NAME__success, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {libo_interfaces__srv__Navigate_Response__FIELD_NAME__message, 7, 7},
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
libo_interfaces__srv__Navigate_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {libo_interfaces__srv__Navigate_Response__TYPE_NAME, 37, 37},
      {libo_interfaces__srv__Navigate_Response__FIELDS, 2, 2},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char libo_interfaces__srv__Navigate_Event__FIELD_NAME__info[] = "info";
static char libo_interfaces__srv__Navigate_Event__FIELD_NAME__request[] = "request";
static char libo_interfaces__srv__Navigate_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field libo_interfaces__srv__Navigate_Event__FIELDS[] = {
  {
    {libo_interfaces__srv__Navigate_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {libo_interfaces__srv__Navigate_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {libo_interfaces__srv__Navigate_Request__TYPE_NAME, 36, 36},
    },
    {NULL, 0, 0},
  },
  {
    {libo_interfaces__srv__Navigate_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {libo_interfaces__srv__Navigate_Response__TYPE_NAME, 37, 37},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription libo_interfaces__srv__Navigate_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {libo_interfaces__srv__Navigate_Request__TYPE_NAME, 36, 36},
    {NULL, 0, 0},
  },
  {
    {libo_interfaces__srv__Navigate_Response__TYPE_NAME, 37, 37},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
libo_interfaces__srv__Navigate_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {libo_interfaces__srv__Navigate_Event__TYPE_NAME, 34, 34},
      {libo_interfaces__srv__Navigate_Event__FIELDS, 3, 3},
    },
    {libo_interfaces__srv__Navigate_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = libo_interfaces__srv__Navigate_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = libo_interfaces__srv__Navigate_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# \\xec\\x9a\\x94\\xec\\xb2\\xad (Request): \\xed\\x81\\xb4\\xeb\\x9d\\xbc\\xec\\x9d\\xb4\\xec\\x96\\xb8\\xed\\x8a\\xb8\\xea\\xb0\\x80 \\xec\\x84\\x9c\\xeb\\xb2\\x84\\xec\\x97\\x90\\xea\\xb2\\x8c \\xeb\\xb3\\xb4\\xeb\\x82\\xb4\\xeb\\x8a\\x94 \\xeb\\x8d\\xb0\\xec\\x9d\\xb4\\xed\\x84\\xb0\n"
  "float32 x     # \\xeb\\xaa\\xa9\\xed\\x91\\x9c x \\xec\\xa2\\x8c\\xed\\x91\\x9c\n"
  "float32 y     # \\xeb\\xaa\\xa9\\xed\\x91\\x9c y \\xec\\xa2\\x8c\\xed\\x91\\x9c\n"
  "float32 yaw   # \\xeb\\xaa\\xa9\\xed\\x91\\x9c yaw \\xea\\xb0\\x81\\xeb\\x8f\\x84 (\\xeb\\xb0\\xa9\\xed\\x96\\xa5)\n"
  "---\n"
  "# \\xec\\x9d\\x91\\xeb\\x8b\\xb5 (Response): \\xec\\x84\\x9c\\xeb\\xb2\\x84\\xea\\xb0\\x80 \\xed\\x81\\xb4\\xeb\\x9d\\xbc\\xec\\x9d\\xb4\\xec\\x96\\xb8\\xed\\x8a\\xb8\\xec\\x97\\x90\\xea\\xb2\\x8c \\xeb\\xb3\\xb4\\xeb\\x82\\xb4\\xeb\\x8a\\x94 \\xea\\xb2\\xb0\\xea\\xb3\\xbc\n"
  "bool success  # \\xec\\x84\\xb1\\xea\\xb3\\xb5 \\xec\\x97\\xac\\xeb\\xb6\\x80\n"
  "string message # \\xea\\xb2\\xb0\\xea\\xb3\\xbc \\xeb\\xa9\\x94\\xec\\x8b\\x9c\\xec\\xa7\\x80";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
libo_interfaces__srv__Navigate__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {libo_interfaces__srv__Navigate__TYPE_NAME, 28, 28},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 200, 200},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
libo_interfaces__srv__Navigate_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {libo_interfaces__srv__Navigate_Request__TYPE_NAME, 36, 36},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
libo_interfaces__srv__Navigate_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {libo_interfaces__srv__Navigate_Response__TYPE_NAME, 37, 37},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
libo_interfaces__srv__Navigate_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {libo_interfaces__srv__Navigate_Event__TYPE_NAME, 34, 34},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
libo_interfaces__srv__Navigate__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *libo_interfaces__srv__Navigate__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *libo_interfaces__srv__Navigate_Event__get_individual_type_description_source(NULL);
    sources[3] = *libo_interfaces__srv__Navigate_Request__get_individual_type_description_source(NULL);
    sources[4] = *libo_interfaces__srv__Navigate_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
libo_interfaces__srv__Navigate_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *libo_interfaces__srv__Navigate_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
libo_interfaces__srv__Navigate_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *libo_interfaces__srv__Navigate_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
libo_interfaces__srv__Navigate_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *libo_interfaces__srv__Navigate_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *libo_interfaces__srv__Navigate_Request__get_individual_type_description_source(NULL);
    sources[3] = *libo_interfaces__srv__Navigate_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
