cmake_minimum_required(VERSION 3.20)
project(HeiHGM_BMatching LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ------------------------------------------------------------------------------
# Options
# ------------------------------------------------------------------------------
option(BMATCHING_USE_GUROBI       "Enable Gurobi ILP solver"                OFF)
option(BMATCHING_USE_SCIP         "Enable SCIP solver"                      OFF)
option(BMATCHING_USE_BSUITOR      "Enable bSuitor algorithm"                OFF)
option(BMATCHING_USE_KARP_SIPSER  "Enable Karp-Sipser algorithm"            OFF)
option(BMATCHING_USE_HASHING      "Enable hashing support (wide-integer)"   OFF)
option(BMATCHING_USE_TCMALLOC     "Use tcmalloc from gperftools"            OFF)
option(BMATCHING_ENABLE_LOGGING   "Enable easylogging++ logging output"     OFF)
option(BMATCHING_FREE_MEMORY_CHECK "Enable free memory checking in runner"  OFF)
option(BUILD_TESTING              "Build tests"                             ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# ------------------------------------------------------------------------------
# FetchContent – external dependencies
# ------------------------------------------------------------------------------
include(FetchContent)

# Abseil
set(ABSL_PROPAGATE_CXX_STD ON CACHE BOOL "" FORCE)
set(ABSL_BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  absl
  GIT_REPOSITORY https://github.com/abseil/abseil-cpp
  GIT_TAG        b971ac5250ea8de900eae9f95e06548d14cd95fe
  GIT_SHALLOW    FALSE
)

# Protobuf
set(protobuf_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(protobuf_BUILD_EXPORT OFF CACHE BOOL "" FORCE)
set(protobuf_INSTALL OFF CACHE BOOL "" FORCE)
set(protobuf_BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  protobuf
  GIT_REPOSITORY https://github.com/protocolbuffers/protobuf
  GIT_TAG        v3.20.3
  SOURCE_SUBDIR  cmake
)

# GoogleTest
if(BUILD_TESTING)
  FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/58d77fa8070e8cec2dc1ed015d66b454c8d78850.zip
  )
endif()

# Easylogging++
FetchContent_Declare(
  easyloggingpp
  GIT_REPOSITORY https://github.com/abumq/easyloggingpp
  GIT_TAG        3bbb9a563858915b9624485356fcc7e0fdb4d68f
)

# Wide integer (header-only)
FetchContent_Declare(
  wide_integer
  GIT_REPOSITORY https://github.com/ckormanyos/wide-integer
  GIT_TAG        d4ab8f42402d26bade2901fcc83c58a2bb9c5037
)

# Optional: pothen_b_matching (bSuitor)
if(BMATCHING_USE_BSUITOR)
  FetchContent_Declare(
    pothen_b_matching
    GIT_REPOSITORY https://github.com/ECP-ExaGraph/bMatching
    GIT_TAG        fee845cf7949a4947197cc93487634276bf69925
  )
endif()

# Optional: ucar_matching (Karp-Sipser)
if(BMATCHING_USE_KARP_SIPSER)
  FetchContent_Declare(
    ucar_matching
    GIT_REPOSITORY https://gitlab.inria.fr/bora-ucar/karp-sipser-for-hypergraphs.git
    GIT_TAG        215f6887d7a3ec843ed70f3e9ea92a65e91a284a
  )
endif()

# Fetch all declared dependencies
FetchContent_MakeAvailable(absl protobuf easyloggingpp wide_integer)

if(BUILD_TESTING)
  FetchContent_MakeAvailable(googletest)
  enable_testing()
endif()

if(BMATCHING_USE_BSUITOR)
  FetchContent_Populate(pothen_b_matching)
endif()

if(BMATCHING_USE_KARP_SIPSER)
  FetchContent_Populate(ucar_matching)
endif()

# ------------------------------------------------------------------------------
# Easylogging++ library target
# ------------------------------------------------------------------------------
add_library(easyloggingpp_lib
  ${easyloggingpp_SOURCE_DIR}/src/easylogging++.cc
)
target_include_directories(easyloggingpp_lib PUBLIC ${easyloggingpp_SOURCE_DIR}/src)
target_compile_definitions(easyloggingpp_lib PUBLIC
  ELPP_FEATURE_PERFORMANCE_TRACKING
  ELPP_NO_LOG_TO_FILE
)
if(NOT BMATCHING_ENABLE_LOGGING)
  target_compile_definitions(easyloggingpp_lib PUBLIC
    ELPP_DISABLE_LOGS
    ELPP_DISABLE_PERFORMANCE_TRACKING
  )
endif()

# ------------------------------------------------------------------------------
# Wide integer interface library
# ------------------------------------------------------------------------------
add_library(wide_integer_lib INTERFACE)
target_include_directories(wide_integer_lib INTERFACE ${wide_integer_SOURCE_DIR})

# ------------------------------------------------------------------------------
# Optional: pothen_b_matching library
# ------------------------------------------------------------------------------
if(BMATCHING_USE_BSUITOR)
  add_library(pothen_bmatching_lib
    ${pothen_b_matching_SOURCE_DIR}/bSuitor.cpp
    ${pothen_b_matching_SOURCE_DIR}/bSuitorD.cpp
    ${pothen_b_matching_SOURCE_DIR}/mtxReader.cpp
  )
  target_include_directories(pothen_bmatching_lib PUBLIC ${pothen_b_matching_SOURCE_DIR})
  target_compile_options(pothen_bmatching_lib PRIVATE -fopenmp -O3)
  set_target_properties(pothen_bmatching_lib PROPERTIES CXX_STANDARD 11)
endif()

# ------------------------------------------------------------------------------
# Optional: ucar_matching libraries
# ------------------------------------------------------------------------------
if(BMATCHING_USE_KARP_SIPSER)
  add_library(ucar_kss_lib ${ucar_matching_SOURCE_DIR}/kss/kss_utils.c)
  target_include_directories(ucar_kss_lib PUBLIC ${ucar_matching_SOURCE_DIR})

  add_library(ucar_ksmd_lib ${ucar_matching_SOURCE_DIR}/ksmd/ksmd_utils.c)
  target_include_directories(ucar_ksmd_lib PUBLIC ${ucar_matching_SOURCE_DIR})
endif()

# ------------------------------------------------------------------------------
# Optional: Gurobi
# ------------------------------------------------------------------------------
if(BMATCHING_USE_GUROBI)
  find_package(Gurobi REQUIRED)
endif()

# ------------------------------------------------------------------------------
# Optional: SCIP
# ------------------------------------------------------------------------------
if(BMATCHING_USE_SCIP)
  find_package(SCIP REQUIRED)
endif()

# ------------------------------------------------------------------------------
# Optional: tcmalloc
# ------------------------------------------------------------------------------
if(BMATCHING_USE_TCMALLOC)
  find_package(Gperftools REQUIRED)
endif()

# ------------------------------------------------------------------------------
# Generate build-info.h
# ------------------------------------------------------------------------------
find_package(Git QUIET)
if(GIT_FOUND)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} tag -l --points-at HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_TAG
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_COMMIT
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --always --dirty
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_DESCRIBE
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
  # Use tag if available, otherwise commit hash
  if(GIT_TAG)
    # Take only the first line if multiple tags
    string(REGEX REPLACE "\n.*" "" GIT_TAG "${GIT_TAG}")
    set(STABLE_VERSION "${GIT_TAG}")
  else()
    set(STABLE_VERSION "${GIT_COMMIT}")
  endif()
  set(STABLE_SCM_DESCRIBE "${GIT_DESCRIBE}")
else()
  set(STABLE_VERSION "unknown")
  set(STABLE_SCM_DESCRIBE "unknown")
endif()

if(BMATCHING_USE_TCMALLOC)
  set(MALLOC_IMPL "tc-malloc")
else()
  set(MALLOC_IMPL "default")
endif()

file(WRITE ${CMAKE_BINARY_DIR}/build-info.h
  "#define STABLE_VERSION \"${STABLE_VERSION}\"\n"
  "#define STABLE_SCM_DESCRIBE \"${STABLE_SCM_DESCRIBE}\"\n"
  "#define MALLOC_IMPLEMENTATION \"${MALLOC_IMPL}\"\n"
)

# ------------------------------------------------------------------------------
# Generate hashing-config.h
# ------------------------------------------------------------------------------
if(BMATCHING_USE_HASHING)
  file(WRITE ${CMAKE_BINARY_DIR}/ds/hashing-config.h "#define HASHING_ACTIVATED 1\n")
else()
  file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/ds)
  file(WRITE ${CMAKE_BINARY_DIR}/ds/hashing-config.h "#undef HASHING_ACTIVATED\n")
endif()

# ------------------------------------------------------------------------------
# Compile app_io.proto
# ------------------------------------------------------------------------------
set(PROTO_SRC ${CMAKE_SOURCE_DIR}/app/app_io.proto)
set(PROTO_GEN_DIR ${CMAKE_BINARY_DIR})

file(MAKE_DIRECTORY ${PROTO_GEN_DIR}/app)

add_custom_command(
  OUTPUT
    ${PROTO_GEN_DIR}/app/app_io.pb.cc
    ${PROTO_GEN_DIR}/app/app_io.pb.h
  COMMAND $<TARGET_FILE:protoc>
    --proto_path=${CMAKE_SOURCE_DIR}
    --proto_path=${protobuf_SOURCE_DIR}/src
    --cpp_out=${PROTO_GEN_DIR}
    ${PROTO_SRC}
  DEPENDS ${PROTO_SRC} protoc
  COMMENT "Generating C++ protobuf sources for app_io.proto"
)

add_library(app_io_cc_proto
  ${PROTO_GEN_DIR}/app/app_io.pb.cc
)
target_include_directories(app_io_cc_proto PUBLIC ${PROTO_GEN_DIR})
target_link_libraries(app_io_cc_proto PUBLIC protobuf::libprotobuf)

# ------------------------------------------------------------------------------
# Global include directories
# The source tree uses includes relative to the project root, e.g.
#   #include "ds/bmatching.h"
#   #include "app/app_io.pb.h"
#   #include "build-info.h"
# ------------------------------------------------------------------------------
include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_BINARY_DIR})

# ------------------------------------------------------------------------------
# Subdirectories
# ------------------------------------------------------------------------------
add_subdirectory(utils)
add_subdirectory(ds)
add_subdirectory(io)
add_subdirectory(bmatching)
add_subdirectory(third_party)
add_subdirectory(app)
add_subdirectory(runner)
add_subdirectory(tools)
