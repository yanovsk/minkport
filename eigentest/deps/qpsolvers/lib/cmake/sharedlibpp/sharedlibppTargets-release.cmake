#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "sharedlibpp::sharedlibpp" for configuration "Release"
set_property(TARGET sharedlibpp::sharedlibpp APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(sharedlibpp::sharedlibpp PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libsharedlibpp.0.0.3.dylib"
  IMPORTED_SONAME_RELEASE "@rpath/libsharedlibpp.1.dylib"
  )

list(APPEND _cmake_import_check_targets sharedlibpp::sharedlibpp )
list(APPEND _cmake_import_check_files_for_sharedlibpp::sharedlibpp "${_IMPORT_PREFIX}/lib/libsharedlibpp.0.0.3.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
