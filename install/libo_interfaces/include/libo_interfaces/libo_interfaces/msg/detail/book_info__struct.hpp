// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from libo_interfaces:msg/BookInfo.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "libo_interfaces/msg/book_info.hpp"


#ifndef LIBO_INTERFACES__MSG__DETAIL__BOOK_INFO__STRUCT_HPP_
#define LIBO_INTERFACES__MSG__DETAIL__BOOK_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__libo_interfaces__msg__BookInfo __attribute__((deprecated))
#else
# define DEPRECATED__libo_interfaces__msg__BookInfo __declspec(deprecated)
#endif

namespace libo_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BookInfo_
{
  using Type = BookInfo_<ContainerAllocator>;

  explicit BookInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->title = "";
      this->author = "";
      this->publisher = "";
      this->category_name = "";
      this->location = "";
      this->price = 0.0;
      this->stock_quantity = 0l;
      this->isbn = "";
      this->cover_image_url = "";
    }
  }

  explicit BookInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : title(_alloc),
    author(_alloc),
    publisher(_alloc),
    category_name(_alloc),
    location(_alloc),
    isbn(_alloc),
    cover_image_url(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->title = "";
      this->author = "";
      this->publisher = "";
      this->category_name = "";
      this->location = "";
      this->price = 0.0;
      this->stock_quantity = 0l;
      this->isbn = "";
      this->cover_image_url = "";
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _title_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _title_type title;
  using _author_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _author_type author;
  using _publisher_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _publisher_type publisher;
  using _category_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _category_name_type category_name;
  using _location_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _location_type location;
  using _price_type =
    double;
  _price_type price;
  using _stock_quantity_type =
    int32_t;
  _stock_quantity_type stock_quantity;
  using _isbn_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _isbn_type isbn;
  using _cover_image_url_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _cover_image_url_type cover_image_url;

  // setters for named parameter idiom
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__title(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->title = _arg;
    return *this;
  }
  Type & set__author(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->author = _arg;
    return *this;
  }
  Type & set__publisher(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->publisher = _arg;
    return *this;
  }
  Type & set__category_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->category_name = _arg;
    return *this;
  }
  Type & set__location(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->location = _arg;
    return *this;
  }
  Type & set__price(
    const double & _arg)
  {
    this->price = _arg;
    return *this;
  }
  Type & set__stock_quantity(
    const int32_t & _arg)
  {
    this->stock_quantity = _arg;
    return *this;
  }
  Type & set__isbn(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->isbn = _arg;
    return *this;
  }
  Type & set__cover_image_url(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->cover_image_url = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    libo_interfaces::msg::BookInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const libo_interfaces::msg::BookInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<libo_interfaces::msg::BookInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<libo_interfaces::msg::BookInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      libo_interfaces::msg::BookInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<libo_interfaces::msg::BookInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      libo_interfaces::msg::BookInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<libo_interfaces::msg::BookInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<libo_interfaces::msg::BookInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<libo_interfaces::msg::BookInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__libo_interfaces__msg__BookInfo
    std::shared_ptr<libo_interfaces::msg::BookInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__libo_interfaces__msg__BookInfo
    std::shared_ptr<libo_interfaces::msg::BookInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BookInfo_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->title != other.title) {
      return false;
    }
    if (this->author != other.author) {
      return false;
    }
    if (this->publisher != other.publisher) {
      return false;
    }
    if (this->category_name != other.category_name) {
      return false;
    }
    if (this->location != other.location) {
      return false;
    }
    if (this->price != other.price) {
      return false;
    }
    if (this->stock_quantity != other.stock_quantity) {
      return false;
    }
    if (this->isbn != other.isbn) {
      return false;
    }
    if (this->cover_image_url != other.cover_image_url) {
      return false;
    }
    return true;
  }
  bool operator!=(const BookInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BookInfo_

// alias to use template instance with default allocator
using BookInfo =
  libo_interfaces::msg::BookInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace libo_interfaces

#endif  // LIBO_INTERFACES__MSG__DETAIL__BOOK_INFO__STRUCT_HPP_
