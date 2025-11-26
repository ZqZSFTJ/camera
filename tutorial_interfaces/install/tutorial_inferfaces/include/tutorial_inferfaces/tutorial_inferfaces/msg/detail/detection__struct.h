// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tutorial_inferfaces:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef TUTORIAL_INFERFACES__MSG__DETAIL__DETECTION__STRUCT_H_
#define TUTORIAL_INFERFACES__MSG__DETAIL__DETECTION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'class_name'
#include "rosidl_runtime_c/string.h"
// Member 'center'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/Detection in the package tutorial_inferfaces.
/**
  * Detection.msg
 */
typedef struct tutorial_inferfaces__msg__Detection
{
  rosidl_runtime_c__String class_name;
  geometry_msgs__msg__Point center;
} tutorial_inferfaces__msg__Detection;

// Struct for a sequence of tutorial_inferfaces__msg__Detection.
typedef struct tutorial_inferfaces__msg__Detection__Sequence
{
  tutorial_inferfaces__msg__Detection * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tutorial_inferfaces__msg__Detection__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TUTORIAL_INFERFACES__MSG__DETAIL__DETECTION__STRUCT_H_
