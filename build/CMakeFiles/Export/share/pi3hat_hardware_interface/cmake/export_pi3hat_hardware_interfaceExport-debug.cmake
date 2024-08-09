#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "pi3hat_hardware_interface::pi3hat_hardware_interface" for configuration "Debug"
set_property(TARGET pi3hat_hardware_interface::pi3hat_hardware_interface APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(pi3hat_hardware_interface::pi3hat_hardware_interface PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libpi3hat_hardware_interface.so"
  IMPORTED_SONAME_DEBUG "libpi3hat_hardware_interface.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS pi3hat_hardware_interface::pi3hat_hardware_interface )
list(APPEND _IMPORT_CHECK_FILES_FOR_pi3hat_hardware_interface::pi3hat_hardware_interface "${_IMPORT_PREFIX}/lib/libpi3hat_hardware_interface.so" )

# Import target "pi3hat_hardware_interface::pi3hat" for configuration "Debug"
set_property(TARGET pi3hat_hardware_interface::pi3hat APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(pi3hat_hardware_interface::pi3hat PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libpi3hat.so"
  IMPORTED_SONAME_DEBUG "libpi3hat.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS pi3hat_hardware_interface::pi3hat )
list(APPEND _IMPORT_CHECK_FILES_FOR_pi3hat_hardware_interface::pi3hat "${_IMPORT_PREFIX}/lib/libpi3hat.so" )

# Import target "pi3hat_hardware_interface::controllers" for configuration "Debug"
set_property(TARGET pi3hat_hardware_interface::controllers APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(pi3hat_hardware_interface::controllers PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libcontrollers.so"
  IMPORTED_SONAME_DEBUG "libcontrollers.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS pi3hat_hardware_interface::controllers )
list(APPEND _IMPORT_CHECK_FILES_FOR_pi3hat_hardware_interface::controllers "${_IMPORT_PREFIX}/lib/libcontrollers.so" )

# Import target "pi3hat_hardware_interface::imu_transform" for configuration "Debug"
set_property(TARGET pi3hat_hardware_interface::imu_transform APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(pi3hat_hardware_interface::imu_transform PROPERTIES
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libimu_transform.so"
  IMPORTED_SONAME_DEBUG "libimu_transform.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS pi3hat_hardware_interface::imu_transform )
list(APPEND _IMPORT_CHECK_FILES_FOR_pi3hat_hardware_interface::imu_transform "${_IMPORT_PREFIX}/lib/libimu_transform.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
