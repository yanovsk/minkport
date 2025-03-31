#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "QpSolversEigen::QpSolversEigen" for configuration "Release"
set_property(TARGET QpSolversEigen::QpSolversEigen APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(QpSolversEigen::QpSolversEigen PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "sharedlibpp::sharedlibpp"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libqpsolvers-eigen.dylib"
  IMPORTED_SONAME_RELEASE "@rpath/libqpsolvers-eigen.dylib"
  )

list(APPEND _cmake_import_check_targets QpSolversEigen::QpSolversEigen )
list(APPEND _cmake_import_check_files_for_QpSolversEigen::QpSolversEigen "${_IMPORT_PREFIX}/lib/libqpsolvers-eigen.dylib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
