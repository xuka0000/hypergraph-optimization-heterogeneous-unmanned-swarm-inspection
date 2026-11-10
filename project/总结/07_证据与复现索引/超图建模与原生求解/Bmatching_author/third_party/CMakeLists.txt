# third_party/CMakeLists.txt

# --- Optional: Karp-Sipser ---
if(BMATCHING_USE_KARP_SIPSER)
  add_library(karp_sipser karp_sipser.cc)
  target_link_libraries(karp_sipser PUBLIC
    app_io_cc_proto
    systeminfo
    absl::statusor
    ucar_kss_lib
    ucar_ksmd_lib
  )
endif()

# --- Optional: bSuitor ---
if(BMATCHING_USE_BSUITOR)
  add_library(bsuitor_lib bsuitor.cc)
  target_link_libraries(bsuitor_lib PUBLIC
    app_io_cc_proto
    systeminfo
    pothen_bmatching_lib
  )
  target_compile_options(bsuitor_lib PRIVATE -fopenmp)
  target_link_options(bsuitor_lib PUBLIC -fopenmp)
endif()

# --- external_graph_bmatching ---
# This is a header-only library with conditional compilation.
# The Bazel build marks it alwayslink=1, so we make it an OBJECT library
# to ensure it is always linked.
add_library(external_graph_bmatching INTERFACE)
target_link_libraries(external_graph_bmatching INTERFACE
  app_io_cc_proto
  absl::statusor
)

if(BMATCHING_USE_BSUITOR)
  target_compile_definitions(external_graph_bmatching INTERFACE HEIHYPBMATCH_USE_BSUITOR)
  target_link_libraries(external_graph_bmatching INTERFACE bsuitor_lib)
endif()

if(BMATCHING_USE_KARP_SIPSER)
  target_compile_definitions(external_graph_bmatching INTERFACE HEIHYPBMATCH_USE_KARP_SIPSER)
  target_link_libraries(external_graph_bmatching INTERFACE karp_sipser)
endif()
