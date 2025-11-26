// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tutorial_inferfaces:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef TUTORIAL_INFERFACES__MSG__DETAIL__DETECTION__TRAITS_HPP_
#define TUTORIAL_INFERFACES__MSG__DETAIL__DETECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tutorial_inferfaces/msg/detail/detection__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'center'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace tutorial_inferfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Detection & msg,
  std::ostream & out)
{
  out << "{";
  // member: class_name
  {
    out << "class_name: ";
    rosidl_generator_traits::value_to_yaml(msg.class_name, out);
    out << ", ";
  }

  // member: center
  {
    out << "center: ";
    to_flow_style_yaml(msg.center, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Detection & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: class_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "class_name: ";
    rosidl_generator_traits::value_to_yaml(msg.class_name, out);
    out << "\n";
  }

  // member: center
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "center:\n";
    to_block_style_yaml(msg.center, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Detection & msg, bool use_flow_style = false)
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

}  // namespace tutorial_inferfaces

namespace rosidl_generator_traits
{

[[deprecated("use tutorial_inferfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tutorial_inferfaces::msg::Detection & msg,
  std::ostream & out, size_t indentation = 0)
{
  tutorial_inferfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tutorial_inferfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const tutorial_inferfaces::msg::Detection & msg)
{
  return tutorial_inferfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tutorial_inferfaces::msg::Detection>()
{
  return "tutorial_inferfaces::msg::Detection";
}

template<>
inline const char * name<tutorial_inferfaces::msg::Detection>()
{
  return "tutorial_inferfaces/msg/Detection";
}

template<>
struct has_fixed_size<tutorial_inferfaces::msg::Detection>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tutorial_inferfaces::msg::Detection>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tutorial_inferfaces::msg::Detection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TUTORIAL_INFERFACES__MSG__DETAIL__DETECTION__TRAITS_HPP_
