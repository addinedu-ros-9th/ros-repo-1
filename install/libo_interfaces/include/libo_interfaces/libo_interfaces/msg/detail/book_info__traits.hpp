// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from libo_interfaces:msg/BookInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "libo_interfaces/msg/book_info.hpp"


#ifndef LIBO_INTERFACES__MSG__DETAIL__BOOK_INFO__TRAITS_HPP_
#define LIBO_INTERFACES__MSG__DETAIL__BOOK_INFO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "libo_interfaces/msg/detail/book_info__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace libo_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const BookInfo & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: title
  {
    out << "title: ";
    rosidl_generator_traits::value_to_yaml(msg.title, out);
    out << ", ";
  }

  // member: author
  {
    out << "author: ";
    rosidl_generator_traits::value_to_yaml(msg.author, out);
    out << ", ";
  }

  // member: publisher
  {
    out << "publisher: ";
    rosidl_generator_traits::value_to_yaml(msg.publisher, out);
    out << ", ";
  }

  // member: category_name
  {
    out << "category_name: ";
    rosidl_generator_traits::value_to_yaml(msg.category_name, out);
    out << ", ";
  }

  // member: location
  {
    out << "location: ";
    rosidl_generator_traits::value_to_yaml(msg.location, out);
    out << ", ";
  }

  // member: price
  {
    out << "price: ";
    rosidl_generator_traits::value_to_yaml(msg.price, out);
    out << ", ";
  }

  // member: stock_quantity
  {
    out << "stock_quantity: ";
    rosidl_generator_traits::value_to_yaml(msg.stock_quantity, out);
    out << ", ";
  }

  // member: isbn
  {
    out << "isbn: ";
    rosidl_generator_traits::value_to_yaml(msg.isbn, out);
    out << ", ";
  }

  // member: cover_image_url
  {
    out << "cover_image_url: ";
    rosidl_generator_traits::value_to_yaml(msg.cover_image_url, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BookInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: title
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "title: ";
    rosidl_generator_traits::value_to_yaml(msg.title, out);
    out << "\n";
  }

  // member: author
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "author: ";
    rosidl_generator_traits::value_to_yaml(msg.author, out);
    out << "\n";
  }

  // member: publisher
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "publisher: ";
    rosidl_generator_traits::value_to_yaml(msg.publisher, out);
    out << "\n";
  }

  // member: category_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "category_name: ";
    rosidl_generator_traits::value_to_yaml(msg.category_name, out);
    out << "\n";
  }

  // member: location
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "location: ";
    rosidl_generator_traits::value_to_yaml(msg.location, out);
    out << "\n";
  }

  // member: price
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "price: ";
    rosidl_generator_traits::value_to_yaml(msg.price, out);
    out << "\n";
  }

  // member: stock_quantity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stock_quantity: ";
    rosidl_generator_traits::value_to_yaml(msg.stock_quantity, out);
    out << "\n";
  }

  // member: isbn
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "isbn: ";
    rosidl_generator_traits::value_to_yaml(msg.isbn, out);
    out << "\n";
  }

  // member: cover_image_url
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cover_image_url: ";
    rosidl_generator_traits::value_to_yaml(msg.cover_image_url, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BookInfo & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace libo_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use libo_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const libo_interfaces::msg::BookInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  libo_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use libo_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const libo_interfaces::msg::BookInfo & msg)
{
  return libo_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<libo_interfaces::msg::BookInfo>()
{
  return "libo_interfaces::msg::BookInfo";
}

template<>
inline const char * name<libo_interfaces::msg::BookInfo>()
{
  return "libo_interfaces/msg/BookInfo";
}

template<>
struct has_fixed_size<libo_interfaces::msg::BookInfo>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<libo_interfaces::msg::BookInfo>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<libo_interfaces::msg::BookInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // LIBO_INTERFACES__MSG__DETAIL__BOOK_INFO__TRAITS_HPP_
