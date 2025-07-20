// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from libo_interfaces:srv/BookSearch.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "libo_interfaces/srv/book_search.hpp"


#ifndef LIBO_INTERFACES__SRV__DETAIL__BOOK_SEARCH__STRUCT_HPP_
#define LIBO_INTERFACES__SRV__DETAIL__BOOK_SEARCH__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__libo_interfaces__srv__BookSearch_Request __attribute__((deprecated))
#else
# define DEPRECATED__libo_interfaces__srv__BookSearch_Request __declspec(deprecated)
#endif

namespace libo_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct BookSearch_Request_
{
  using Type = BookSearch_Request_<ContainerAllocator>;

  explicit BookSearch_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->query = "";
      this->search_type = "";
    }
  }

  explicit BookSearch_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : query(_alloc),
    search_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->query = "";
      this->search_type = "";
    }
  }

  // field types and members
  using _query_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _query_type query;
  using _search_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _search_type_type search_type;

  // setters for named parameter idiom
  Type & set__query(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->query = _arg;
    return *this;
  }
  Type & set__search_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->search_type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    libo_interfaces::srv::BookSearch_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const libo_interfaces::srv::BookSearch_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<libo_interfaces::srv::BookSearch_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<libo_interfaces::srv::BookSearch_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      libo_interfaces::srv::BookSearch_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<libo_interfaces::srv::BookSearch_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      libo_interfaces::srv::BookSearch_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<libo_interfaces::srv::BookSearch_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<libo_interfaces::srv::BookSearch_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<libo_interfaces::srv::BookSearch_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__libo_interfaces__srv__BookSearch_Request
    std::shared_ptr<libo_interfaces::srv::BookSearch_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__libo_interfaces__srv__BookSearch_Request
    std::shared_ptr<libo_interfaces::srv::BookSearch_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BookSearch_Request_ & other) const
  {
    if (this->query != other.query) {
      return false;
    }
    if (this->search_type != other.search_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const BookSearch_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BookSearch_Request_

// alias to use template instance with default allocator
using BookSearch_Request =
  libo_interfaces::srv::BookSearch_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace libo_interfaces


// Include directives for member types
// Member 'books'
#include "libo_interfaces/msg/detail/book_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__libo_interfaces__srv__BookSearch_Response __attribute__((deprecated))
#else
# define DEPRECATED__libo_interfaces__srv__BookSearch_Response __declspec(deprecated)
#endif

namespace libo_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct BookSearch_Response_
{
  using Type = BookSearch_Response_<ContainerAllocator>;

  explicit BookSearch_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  explicit BookSearch_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
      this->message = "";
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;
  using _message_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _message_type message;
  using _books_type =
    std::vector<libo_interfaces::msg::BookInfo_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<libo_interfaces::msg::BookInfo_<ContainerAllocator>>>;
  _books_type books;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }
  Type & set__message(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->message = _arg;
    return *this;
  }
  Type & set__books(
    const std::vector<libo_interfaces::msg::BookInfo_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<libo_interfaces::msg::BookInfo_<ContainerAllocator>>> & _arg)
  {
    this->books = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    libo_interfaces::srv::BookSearch_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const libo_interfaces::srv::BookSearch_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<libo_interfaces::srv::BookSearch_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<libo_interfaces::srv::BookSearch_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      libo_interfaces::srv::BookSearch_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<libo_interfaces::srv::BookSearch_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      libo_interfaces::srv::BookSearch_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<libo_interfaces::srv::BookSearch_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<libo_interfaces::srv::BookSearch_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<libo_interfaces::srv::BookSearch_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__libo_interfaces__srv__BookSearch_Response
    std::shared_ptr<libo_interfaces::srv::BookSearch_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__libo_interfaces__srv__BookSearch_Response
    std::shared_ptr<libo_interfaces::srv::BookSearch_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BookSearch_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    if (this->message != other.message) {
      return false;
    }
    if (this->books != other.books) {
      return false;
    }
    return true;
  }
  bool operator!=(const BookSearch_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BookSearch_Response_

// alias to use template instance with default allocator
using BookSearch_Response =
  libo_interfaces::srv::BookSearch_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace libo_interfaces


// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__libo_interfaces__srv__BookSearch_Event __attribute__((deprecated))
#else
# define DEPRECATED__libo_interfaces__srv__BookSearch_Event __declspec(deprecated)
#endif

namespace libo_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct BookSearch_Event_
{
  using Type = BookSearch_Event_<ContainerAllocator>;

  explicit BookSearch_Event_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_init)
  {
    (void)_init;
  }

  explicit BookSearch_Event_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _info_type =
    service_msgs::msg::ServiceEventInfo_<ContainerAllocator>;
  _info_type info;
  using _request_type =
    rosidl_runtime_cpp::BoundedVector<libo_interfaces::srv::BookSearch_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<libo_interfaces::srv::BookSearch_Request_<ContainerAllocator>>>;
  _request_type request;
  using _response_type =
    rosidl_runtime_cpp::BoundedVector<libo_interfaces::srv::BookSearch_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<libo_interfaces::srv::BookSearch_Response_<ContainerAllocator>>>;
  _response_type response;

  // setters for named parameter idiom
  Type & set__info(
    const service_msgs::msg::ServiceEventInfo_<ContainerAllocator> & _arg)
  {
    this->info = _arg;
    return *this;
  }
  Type & set__request(
    const rosidl_runtime_cpp::BoundedVector<libo_interfaces::srv::BookSearch_Request_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<libo_interfaces::srv::BookSearch_Request_<ContainerAllocator>>> & _arg)
  {
    this->request = _arg;
    return *this;
  }
  Type & set__response(
    const rosidl_runtime_cpp::BoundedVector<libo_interfaces::srv::BookSearch_Response_<ContainerAllocator>, 1, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<libo_interfaces::srv::BookSearch_Response_<ContainerAllocator>>> & _arg)
  {
    this->response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    libo_interfaces::srv::BookSearch_Event_<ContainerAllocator> *;
  using ConstRawPtr =
    const libo_interfaces::srv::BookSearch_Event_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<libo_interfaces::srv::BookSearch_Event_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<libo_interfaces::srv::BookSearch_Event_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      libo_interfaces::srv::BookSearch_Event_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<libo_interfaces::srv::BookSearch_Event_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      libo_interfaces::srv::BookSearch_Event_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<libo_interfaces::srv::BookSearch_Event_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<libo_interfaces::srv::BookSearch_Event_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<libo_interfaces::srv::BookSearch_Event_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__libo_interfaces__srv__BookSearch_Event
    std::shared_ptr<libo_interfaces::srv::BookSearch_Event_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__libo_interfaces__srv__BookSearch_Event
    std::shared_ptr<libo_interfaces::srv::BookSearch_Event_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BookSearch_Event_ & other) const
  {
    if (this->info != other.info) {
      return false;
    }
    if (this->request != other.request) {
      return false;
    }
    if (this->response != other.response) {
      return false;
    }
    return true;
  }
  bool operator!=(const BookSearch_Event_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BookSearch_Event_

// alias to use template instance with default allocator
using BookSearch_Event =
  libo_interfaces::srv::BookSearch_Event_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace libo_interfaces

namespace libo_interfaces
{

namespace srv
{

struct BookSearch
{
  using Request = libo_interfaces::srv::BookSearch_Request;
  using Response = libo_interfaces::srv::BookSearch_Response;
  using Event = libo_interfaces::srv::BookSearch_Event;
};

}  // namespace srv

}  // namespace libo_interfaces

#endif  // LIBO_INTERFACES__SRV__DETAIL__BOOK_SEARCH__STRUCT_HPP_
