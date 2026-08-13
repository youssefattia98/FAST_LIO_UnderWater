# Locate the public GTSAM shared library without importing distro-specific
# test and private dependency targets. Some Ubuntu GTSAM packages export
# CppUnitLite and unversioned SuiteSparse targets that are not shipped by the
# runtime package, although libgtsam itself is complete.

find_path(GTSAM_INCLUDE_DIR
  NAMES gtsam/config.h
  DOC "GTSAM include directory")

find_library(GTSAM_LIBRARY
  NAMES gtsam
  DOC "GTSAM shared library")

set(GTSAM_VERSION "")
if(GTSAM_INCLUDE_DIR AND EXISTS "${GTSAM_INCLUDE_DIR}/gtsam/config.h")
  file(STRINGS "${GTSAM_INCLUDE_DIR}/gtsam/config.h" _gtsam_version_lines
    REGEX "^#define GTSAM_VERSION_(MAJOR|MINOR|PATCH) [0-9]+$")
  foreach(_component MAJOR MINOR PATCH)
    foreach(_line IN LISTS _gtsam_version_lines)
      if(_line MATCHES "^#define GTSAM_VERSION_${_component} ([0-9]+)$")
        set(GTSAM_VERSION_${_component} "${CMAKE_MATCH_1}")
      endif()
    endforeach()
  endforeach()
  if(DEFINED GTSAM_VERSION_MAJOR AND DEFINED GTSAM_VERSION_MINOR AND
     DEFINED GTSAM_VERSION_PATCH)
    set(GTSAM_VERSION
      "${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}.${GTSAM_VERSION_PATCH}")
  endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GTSAM
  REQUIRED_VARS GTSAM_LIBRARY GTSAM_INCLUDE_DIR
  VERSION_VAR GTSAM_VERSION)

if(GTSAM_FOUND AND NOT TARGET gtsam)
  add_library(gtsam UNKNOWN IMPORTED GLOBAL)
  set_target_properties(gtsam PROPERTIES
    IMPORTED_LOCATION "${GTSAM_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${GTSAM_INCLUDE_DIR}")
endif()

mark_as_advanced(GTSAM_INCLUDE_DIR GTSAM_LIBRARY)
