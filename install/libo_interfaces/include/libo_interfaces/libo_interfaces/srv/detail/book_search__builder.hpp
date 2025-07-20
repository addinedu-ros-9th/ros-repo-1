// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from libo_interfaces:srv/BookSearch.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "libo_interfaces/srv/book_search.hpp"


#ifndef LIBO_INTERFACES__SRV__DETAIL__BOOK_SEARCH__BUILDER_HPP_
#define LIBO_INTERFACES__SRV__DETAIL__BOOK_SEARCH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "libo_interfaces/srv/detail/book_search__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace libo_interfaces
{

namespace srv
{

namespace builder
{

class Init_BookSearch_Request_search_type
{
public:
  explicit Init_BookSearch_Request_search_type(::libo_interfaces::srv::BookSearch_Request & msg)
  : msg_(msg)
  {}
  ::libo_interfaces::srv::BookSearch_Request search_type(::libo_interfaces::srv::BookSearch_Request::_search_type_type arg)
  {
    msg_.search_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::libo_interfaces::srv::BookSearch_Request msg_;
};

class Init_BookSearch_Request_query
{
public:
  Init_BookSearch_Request_query()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BookSearch_Request_search_type query(::libo_interfaces::srv::BookSearch_Request::_query_type arg)
  {
    msg_.query = std::move(arg);
    return Init_BookSearch_Request_search_type(msg_);
  }

private:
  ::libo_interfaces::srv::BookSearch_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::libo_interfaces::srv::BookSearch_Request>()
{
  return libo_interfaces::srv::builder::Init_BookSearch_Request_query();
}

}  // namespace libo_interfaces


namespace libo_interfaces
{

namespace srv
{

namespace builder
{

class Init_BookSearch_Response_books
{
public:
  explicit Init_BookSearch_Response_books(::libo_interfaces::srv::BookSearch_Response & msg)
  : msg_(msg)
  {}
  ::libo_interfaces::srv::BookSearch_Response books(::libo_interfaces::srv::BookSearch_Response::_books_type arg)
  {
    msg_.books = std::move(arg);
    return std::move(msg_);
  }

private:
  ::libo_interfaces::srv::BookSearch_Response msg_;
};

class Init_BookSearch_Response_message
{
public:
  explicit Init_BookSearch_Response_message(::libo_interfaces::srv::BookSearch_Response & msg)
  : msg_(msg)
  {}
  Init_BookSearch_Response_books message(::libo_interfaces::srv::BookSearch_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return Init_BookSearch_Response_books(msg_);
  }

private:
  ::libo_interfaces::srv::BookSearch_Response msg_;
};

class Init_BookSearch_Response_success
{
public:
  Init_BookSearch_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BookSearch_Response_message success(::libo_interfaces::srv::BookSearch_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_BookSearch_Response_message(msg_);
  }

private:
  ::libo_interfaces::srv::BookSearch_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::libo_interfaces::srv::BookSearch_Response>()
{
  return libo_interfaces::srv::builder::Init_BookSearch_Response_success();
}

}  // namespace libo_interfaces


namespace libo_interfaces
{

namespace srv
{

namespace builder
{

class Init_BookSearch_Event_response
{
public:
  explicit Init_BookSearch_Event_response(::libo_interfaces::srv::BookSearch_Event & msg)
  : msg_(msg)
  {}
  ::libo_interfaces::srv::BookSearch_Event response(::libo_interfaces::srv::BookSearch_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::libo_interfaces::srv::BookSearch_Event msg_;
};

class Init_BookSearch_Event_request
{
public:
  explicit Init_BookSearch_Event_request(::libo_interfaces::srv::BookSearch_Event & msg)
  : msg_(msg)
  {}
  Init_BookSearch_Event_response request(::libo_interfaces::srv::BookSearch_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_BookSearch_Event_response(msg_);
  }

private:
  ::libo_interfaces::srv::BookSearch_Event msg_;
};

class Init_BookSearch_Event_info
{
public:
  Init_BookSearch_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BookSearch_Event_request info(::libo_interfaces::srv::BookSearch_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_BookSearch_Event_request(msg_);
  }

private:
  ::libo_interfaces::srv::BookSearch_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::libo_interfaces::srv::BookSearch_Event>()
{
  return libo_interfaces::srv::builder::Init_BookSearch_Event_info();
}

}  // namespace libo_interfaces

#endif  // LIBO_INTERFACES__SRV__DETAIL__BOOK_SEARCH__BUILDER_HPP_
