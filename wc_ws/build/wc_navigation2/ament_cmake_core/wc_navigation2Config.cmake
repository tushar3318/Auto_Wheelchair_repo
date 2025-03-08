# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_wc_navigation2_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED wc_navigation2_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(wc_navigation2_FOUND FALSE)
  elseif(NOT wc_navigation2_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(wc_navigation2_FOUND FALSE)
  endif()
  return()
endif()
set(_wc_navigation2_CONFIG_INCLUDED TRUE)

# output package information
if(NOT wc_navigation2_FIND_QUIETLY)
  message(STATUS "Found wc_navigation2: 0.0.1 (${wc_navigation2_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'wc_navigation2' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${wc_navigation2_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(wc_navigation2_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${wc_navigation2_DIR}/${_extra}")
endforeach()
