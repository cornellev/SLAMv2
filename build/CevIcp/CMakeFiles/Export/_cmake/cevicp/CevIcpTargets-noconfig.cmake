#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "CEV::cevicp" for configuration ""
set_property(TARGET CEV::cevicp APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(CEV::cevicp PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libcevicp.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS CEV::cevicp )
list(APPEND _IMPORT_CHECK_FILES_FOR_CEV::cevicp "${_IMPORT_PREFIX}/lib/libcevicp.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
