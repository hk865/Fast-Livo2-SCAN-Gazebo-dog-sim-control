#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "traj_utils::traj_utils" for configuration "Release"
set_property(TARGET traj_utils::traj_utils APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(traj_utils::traj_utils PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libtraj_utils.a"
  )

list(APPEND _cmake_import_check_targets traj_utils::traj_utils )
list(APPEND _cmake_import_check_files_for_traj_utils::traj_utils "${_IMPORT_PREFIX}/lib/libtraj_utils.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
