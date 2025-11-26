# generated from
# rosidl_cmake/cmake/template/rosidl_cmake_export_typesupport_targets.cmake.in

set(_exported_typesupport_targets
  "__rosidl_generator_c:tutorial_inferfaces__rosidl_generator_c;__rosidl_typesupport_fastrtps_c:tutorial_inferfaces__rosidl_typesupport_fastrtps_c;__rosidl_generator_cpp:tutorial_inferfaces__rosidl_generator_cpp;__rosidl_typesupport_fastrtps_cpp:tutorial_inferfaces__rosidl_typesupport_fastrtps_cpp;__rosidl_typesupport_introspection_c:tutorial_inferfaces__rosidl_typesupport_introspection_c;__rosidl_typesupport_c:tutorial_inferfaces__rosidl_typesupport_c;__rosidl_typesupport_introspection_cpp:tutorial_inferfaces__rosidl_typesupport_introspection_cpp;__rosidl_typesupport_cpp:tutorial_inferfaces__rosidl_typesupport_cpp;__rosidl_generator_py:tutorial_inferfaces__rosidl_generator_py")

# populate tutorial_inferfaces_TARGETS_<suffix>
if(NOT _exported_typesupport_targets STREQUAL "")
  # loop over typesupport targets
  foreach(_tuple ${_exported_typesupport_targets})
    string(REPLACE ":" ";" _tuple "${_tuple}")
    list(GET _tuple 0 _suffix)
    list(GET _tuple 1 _target)

    set(_target "tutorial_inferfaces::${_target}")
    if(NOT TARGET "${_target}")
      # the exported target must exist
      message(WARNING "Package 'tutorial_inferfaces' exports the typesupport target '${_target}' which doesn't exist")
    else()
      list(APPEND tutorial_inferfaces_TARGETS${_suffix} "${_target}")
    endif()
  endforeach()
endif()
