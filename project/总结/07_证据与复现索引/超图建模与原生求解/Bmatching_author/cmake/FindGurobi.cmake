# FindGurobi.cmake
# Finds the Gurobi optimizer library.
#
# Sets:
#   Gurobi_FOUND
#   Gurobi::Gurobi (imported target)
#
# Uses GUROBI_HOME environment variable or CMake variable as hint.

if(NOT GUROBI_HOME)
  set(GUROBI_HOME "$ENV{GUROBI_HOME}")
endif()

if(NOT GUROBI_HOME)
  set(GUROBI_HOME "/opt/gurobi951/linux64")
endif()

find_path(GUROBI_INCLUDE_DIR
  NAMES gurobi_c.h
  HINTS ${GUROBI_HOME}/include
)

# Search for versioned Gurobi C library (gurobi100, gurobi95, etc.)
file(GLOB _gurobi_c_libs "${GUROBI_HOME}/lib/libgurobi*.so")
# Filter out C++ wrapper
list(FILTER _gurobi_c_libs EXCLUDE REGEX "gurobi_c\\+\\+")

if(_gurobi_c_libs)
  list(GET _gurobi_c_libs 0 GUROBI_C_LIBRARY)
endif()

find_library(GUROBI_CXX_LIBRARY
  NAMES gurobi_c++
  HINTS ${GUROBI_HOME}/lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Gurobi
  REQUIRED_VARS GUROBI_INCLUDE_DIR GUROBI_C_LIBRARY GUROBI_CXX_LIBRARY
)

if(Gurobi_FOUND AND NOT TARGET Gurobi::Gurobi)
  add_library(Gurobi::Gurobi INTERFACE IMPORTED)
  set_target_properties(Gurobi::Gurobi PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${GUROBI_INCLUDE_DIR}"
    INTERFACE_LINK_LIBRARIES "${GUROBI_CXX_LIBRARY};${GUROBI_C_LIBRARY}"
  )
endif()
