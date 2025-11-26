// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tutorial_inferfaces:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef TUTORIAL_INFERFACES__MSG__DETAIL__DETECTION__BUILDER_HPP_
#define TUTORIAL_INFERFACES__MSG__DETAIL__DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tutorial_inferfaces/msg/detail/detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tutorial_inferfaces
{

namespace msg
{

namespace builder
{

class Init_Detection_center
{
public:
  explicit Init_Detection_center(::tutorial_inferfaces::msg::Detection & msg)
  : msg_(msg)
  {}
  ::tutorial_inferfaces::msg::Detection center(::tutorial_inferfaces::msg::Detection::_center_type arg)
  {
    msg_.center = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tutorial_inferfaces::msg::Detection msg_;
};

class Init_Detection_class_name
{
public:
  Init_Detection_class_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Detection_center class_name(::tutorial_inferfaces::msg::Detection::_class_name_type arg)
  {
    msg_.class_name = std::move(arg);
    return Init_Detection_center(msg_);
  }

private:
  ::tutorial_inferfaces::msg::Detection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tutorial_inferfaces::msg::Detection>()
{
  return tutorial_inferfaces::msg::builder::Init_Detection_class_name();
}

}  // namespace tutorial_inferfaces

#endif  // TUTORIAL_INFERFACES__MSG__DETAIL__DETECTION__BUILDER_HPP_
