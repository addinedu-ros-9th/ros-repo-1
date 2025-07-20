// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from libo_interfaces:msg/BookInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "libo_interfaces/msg/book_info.hpp"


#ifndef LIBO_INTERFACES__MSG__DETAIL__BOOK_INFO__BUILDER_HPP_
#define LIBO_INTERFACES__MSG__DETAIL__BOOK_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "libo_interfaces/msg/detail/book_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace libo_interfaces
{

namespace msg
{

namespace builder
{

class Init_BookInfo_cover_image_url
{
public:
  explicit Init_BookInfo_cover_image_url(::libo_interfaces::msg::BookInfo & msg)
  : msg_(msg)
  {}
  ::libo_interfaces::msg::BookInfo cover_image_url(::libo_interfaces::msg::BookInfo::_cover_image_url_type arg)
  {
    msg_.cover_image_url = std::move(arg);
    return std::move(msg_);
  }

private:
  ::libo_interfaces::msg::BookInfo msg_;
};

class Init_BookInfo_isbn
{
public:
  explicit Init_BookInfo_isbn(::libo_interfaces::msg::BookInfo & msg)
  : msg_(msg)
  {}
  Init_BookInfo_cover_image_url isbn(::libo_interfaces::msg::BookInfo::_isbn_type arg)
  {
    msg_.isbn = std::move(arg);
    return Init_BookInfo_cover_image_url(msg_);
  }

private:
  ::libo_interfaces::msg::BookInfo msg_;
};

class Init_BookInfo_stock_quantity
{
public:
  explicit Init_BookInfo_stock_quantity(::libo_interfaces::msg::BookInfo & msg)
  : msg_(msg)
  {}
  Init_BookInfo_isbn stock_quantity(::libo_interfaces::msg::BookInfo::_stock_quantity_type arg)
  {
    msg_.stock_quantity = std::move(arg);
    return Init_BookInfo_isbn(msg_);
  }

private:
  ::libo_interfaces::msg::BookInfo msg_;
};

class Init_BookInfo_price
{
public:
  explicit Init_BookInfo_price(::libo_interfaces::msg::BookInfo & msg)
  : msg_(msg)
  {}
  Init_BookInfo_stock_quantity price(::libo_interfaces::msg::BookInfo::_price_type arg)
  {
    msg_.price = std::move(arg);
    return Init_BookInfo_stock_quantity(msg_);
  }

private:
  ::libo_interfaces::msg::BookInfo msg_;
};

class Init_BookInfo_location
{
public:
  explicit Init_BookInfo_location(::libo_interfaces::msg::BookInfo & msg)
  : msg_(msg)
  {}
  Init_BookInfo_price location(::libo_interfaces::msg::BookInfo::_location_type arg)
  {
    msg_.location = std::move(arg);
    return Init_BookInfo_price(msg_);
  }

private:
  ::libo_interfaces::msg::BookInfo msg_;
};

class Init_BookInfo_category_name
{
public:
  explicit Init_BookInfo_category_name(::libo_interfaces::msg::BookInfo & msg)
  : msg_(msg)
  {}
  Init_BookInfo_location category_name(::libo_interfaces::msg::BookInfo::_category_name_type arg)
  {
    msg_.category_name = std::move(arg);
    return Init_BookInfo_location(msg_);
  }

private:
  ::libo_interfaces::msg::BookInfo msg_;
};

class Init_BookInfo_publisher
{
public:
  explicit Init_BookInfo_publisher(::libo_interfaces::msg::BookInfo & msg)
  : msg_(msg)
  {}
  Init_BookInfo_category_name publisher(::libo_interfaces::msg::BookInfo::_publisher_type arg)
  {
    msg_.publisher = std::move(arg);
    return Init_BookInfo_category_name(msg_);
  }

private:
  ::libo_interfaces::msg::BookInfo msg_;
};

class Init_BookInfo_author
{
public:
  explicit Init_BookInfo_author(::libo_interfaces::msg::BookInfo & msg)
  : msg_(msg)
  {}
  Init_BookInfo_publisher author(::libo_interfaces::msg::BookInfo::_author_type arg)
  {
    msg_.author = std::move(arg);
    return Init_BookInfo_publisher(msg_);
  }

private:
  ::libo_interfaces::msg::BookInfo msg_;
};

class Init_BookInfo_title
{
public:
  explicit Init_BookInfo_title(::libo_interfaces::msg::BookInfo & msg)
  : msg_(msg)
  {}
  Init_BookInfo_author title(::libo_interfaces::msg::BookInfo::_title_type arg)
  {
    msg_.title = std::move(arg);
    return Init_BookInfo_author(msg_);
  }

private:
  ::libo_interfaces::msg::BookInfo msg_;
};

class Init_BookInfo_id
{
public:
  Init_BookInfo_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BookInfo_title id(::libo_interfaces::msg::BookInfo::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_BookInfo_title(msg_);
  }

private:
  ::libo_interfaces::msg::BookInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::libo_interfaces::msg::BookInfo>()
{
  return libo_interfaces::msg::builder::Init_BookInfo_id();
}

}  // namespace libo_interfaces

#endif  // LIBO_INTERFACES__MSG__DETAIL__BOOK_INFO__BUILDER_HPP_
