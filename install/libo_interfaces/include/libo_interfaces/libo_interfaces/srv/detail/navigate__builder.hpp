// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from libo_interfaces:srv/Navigate.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "libo_interfaces/srv/navigate.hpp"


#ifndef LIBO_INTERFACES__SRV__DETAIL__NAVIGATE__BUILDER_HPP_
#define LIBO_INTERFACES__SRV__DETAIL__NAVIGATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "libo_interfaces/srv/detail/navigate__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace libo_interfaces
{

namespace srv
{

namespace builder
{

class Init_Navigate_Request_yaw
{
public:
  explicit Init_Navigate_Request_yaw(::libo_interfaces::srv::Navigate_Request & msg)
  : msg_(msg)
  {}
  ::libo_interfaces::srv::Navigate_Request yaw(::libo_interfaces::srv::Navigate_Request::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::libo_interfaces::srv::Navigate_Request msg_;
};

class Init_Navigate_Request_y
{
public:
  explicit Init_Navigate_Request_y(::libo_interfaces::srv::Navigate_Request & msg)
  : msg_(msg)
  {}
  Init_Navigate_Request_yaw y(::libo_interfaces::srv::Navigate_Request::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_Navigate_Request_yaw(msg_);
  }

private:
  ::libo_interfaces::srv::Navigate_Request msg_;
};

class Init_Navigate_Request_x
{
public:
  Init_Navigate_Request_x()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Navigate_Request_y x(::libo_interfaces::srv::Navigate_Request::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_Navigate_Request_y(msg_);
  }

private:
  ::libo_interfaces::srv::Navigate_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::libo_interfaces::srv::Navigate_Request>()
{
  return libo_interfaces::srv::builder::Init_Navigate_Request_x();
}

}  // namespace libo_interfaces


namespace libo_interfaces
{

namespace srv
{

namespace builder
{

class Init_Navigate_Response_message
{
public:
  explicit Init_Navigate_Response_message(::libo_interfaces::srv::Navigate_Response & msg)
  : msg_(msg)
  {}
  ::libo_interfaces::srv::Navigate_Response message(::libo_interfaces::srv::Navigate_Response::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::libo_interfaces::srv::Navigate_Response msg_;
};

class Init_Navigate_Response_success
{
public:
  Init_Navigate_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Navigate_Response_message success(::libo_interfaces::srv::Navigate_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_Navigate_Response_message(msg_);
  }

private:
  ::libo_interfaces::srv::Navigate_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::libo_interfaces::srv::Navigate_Response>()
{
  return libo_interfaces::srv::builder::Init_Navigate_Response_success();
}

}  // namespace libo_interfaces


namespace libo_interfaces
{

namespace srv
{

namespace builder
{

class Init_Navigate_Event_response
{
public:
  explicit Init_Navigate_Event_response(::libo_interfaces::srv::Navigate_Event & msg)
  : msg_(msg)
  {}
  ::libo_interfaces::srv::Navigate_Event response(::libo_interfaces::srv::Navigate_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::libo_interfaces::srv::Navigate_Event msg_;
};

class Init_Navigate_Event_request
{
public:
  explicit Init_Navigate_Event_request(::libo_interfaces::srv::Navigate_Event & msg)
  : msg_(msg)
  {}
  Init_Navigate_Event_response request(::libo_interfaces::srv::Navigate_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_Navigate_Event_response(msg_);
  }

private:
  ::libo_interfaces::srv::Navigate_Event msg_;
};

class Init_Navigate_Event_info
{
public:
  Init_Navigate_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Navigate_Event_request info(::libo_interfaces::srv::Navigate_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_Navigate_Event_request(msg_);
  }

private:
  ::libo_interfaces::srv::Navigate_Event msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::libo_interfaces::srv::Navigate_Event>()
{
  return libo_interfaces::srv::builder::Init_Navigate_Event_info();
}

}  // namespace libo_interfaces

#endif  // LIBO_INTERFACES__SRV__DETAIL__NAVIGATE__BUILDER_HPP_
