// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from libo_interfaces:srv/Navigate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "libo_interfaces/srv/navigate.hpp"


#ifndef LIBO_INTERFACES__SRV__DETAIL__NAVIGATE__TRAITS_HPP_
#define LIBO_INTERFACES__SRV__DETAIL__NAVIGATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "libo_interfaces/srv/detail/navigate__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace libo_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Navigate_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: x
  {
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << ", ";
  }

  // member: y
  {
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << ", ";
  }

  // member: yaw
  {
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Navigate_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x: ";
    rosidl_generator_traits::value_to_yaml(msg.x, out);
    out << "\n";
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y: ";
    rosidl_generator_traits::value_to_yaml(msg.y, out);
    out << "\n";
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Navigate_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace libo_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use libo_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const libo_interfaces::srv::Navigate_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  libo_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use libo_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const libo_interfaces::srv::Navigate_Request & msg)
{
  return libo_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<libo_interfaces::srv::Navigate_Request>()
{
  return "libo_interfaces::srv::Navigate_Request";
}

template<>
inline const char * name<libo_interfaces::srv::Navigate_Request>()
{
  return "libo_interfaces/srv/Navigate_Request";
}

template<>
struct has_fixed_size<libo_interfaces::srv::Navigate_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<libo_interfaces::srv::Navigate_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<libo_interfaces::srv::Navigate_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace libo_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Navigate_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << ", ";
  }

  // member: message
  {
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Navigate_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }

  // member: message
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "message: ";
    rosidl_generator_traits::value_to_yaml(msg.message, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Navigate_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace libo_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use libo_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const libo_interfaces::srv::Navigate_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  libo_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use libo_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const libo_interfaces::srv::Navigate_Response & msg)
{
  return libo_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<libo_interfaces::srv::Navigate_Response>()
{
  return "libo_interfaces::srv::Navigate_Response";
}

template<>
inline const char * name<libo_interfaces::srv::Navigate_Response>()
{
  return "libo_interfaces/srv/Navigate_Response";
}

template<>
struct has_fixed_size<libo_interfaces::srv::Navigate_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<libo_interfaces::srv::Navigate_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<libo_interfaces::srv::Navigate_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__traits.hpp"

namespace libo_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const Navigate_Event & msg,
  std::ostream & out)
{
  out << "{";
  // member: info
  {
    out << "info: ";
    to_flow_style_yaml(msg.info, out);
    out << ", ";
  }

  // member: request
  {
    if (msg.request.size() == 0) {
      out << "request: []";
    } else {
      out << "request: [";
      size_t pending_items = msg.request.size();
      for (auto item : msg.request) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: response
  {
    if (msg.response.size() == 0) {
      out << "response: []";
    } else {
      out << "response: [";
      size_t pending_items = msg.response.size();
      for (auto item : msg.response) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Navigate_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "info:\n";
    to_block_style_yaml(msg.info, out, indentation + 2);
  }

  // member: request
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.request.size() == 0) {
      out << "request: []\n";
    } else {
      out << "request:\n";
      for (auto item : msg.request) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.response.size() == 0) {
      out << "response: []\n";
    } else {
      out << "response:\n";
      for (auto item : msg.response) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Navigate_Event & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace libo_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use libo_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const libo_interfaces::srv::Navigate_Event & msg,
  std::ostream & out, size_t indentation = 0)
{
  libo_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use libo_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const libo_interfaces::srv::Navigate_Event & msg)
{
  return libo_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<libo_interfaces::srv::Navigate_Event>()
{
  return "libo_interfaces::srv::Navigate_Event";
}

template<>
inline const char * name<libo_interfaces::srv::Navigate_Event>()
{
  return "libo_interfaces/srv/Navigate_Event";
}

template<>
struct has_fixed_size<libo_interfaces::srv::Navigate_Event>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<libo_interfaces::srv::Navigate_Event>
  : std::integral_constant<bool, has_bounded_size<libo_interfaces::srv::Navigate_Request>::value && has_bounded_size<libo_interfaces::srv::Navigate_Response>::value && has_bounded_size<service_msgs::msg::ServiceEventInfo>::value> {};

template<>
struct is_message<libo_interfaces::srv::Navigate_Event>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<libo_interfaces::srv::Navigate>()
{
  return "libo_interfaces::srv::Navigate";
}

template<>
inline const char * name<libo_interfaces::srv::Navigate>()
{
  return "libo_interfaces/srv/Navigate";
}

template<>
struct has_fixed_size<libo_interfaces::srv::Navigate>
  : std::integral_constant<
    bool,
    has_fixed_size<libo_interfaces::srv::Navigate_Request>::value &&
    has_fixed_size<libo_interfaces::srv::Navigate_Response>::value
  >
{
};

template<>
struct has_bounded_size<libo_interfaces::srv::Navigate>
  : std::integral_constant<
    bool,
    has_bounded_size<libo_interfaces::srv::Navigate_Request>::value &&
    has_bounded_size<libo_interfaces::srv::Navigate_Response>::value
  >
{
};

template<>
struct is_service<libo_interfaces::srv::Navigate>
  : std::true_type
{
};

template<>
struct is_service_request<libo_interfaces::srv::Navigate_Request>
  : std::true_type
{
};

template<>
struct is_service_response<libo_interfaces::srv::Navigate_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // LIBO_INTERFACES__SRV__DETAIL__NAVIGATE__TRAITS_HPP_
