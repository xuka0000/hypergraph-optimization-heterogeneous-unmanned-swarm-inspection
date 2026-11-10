# app/algorithms/CMakeLists.txt

set(ALGORITHM_IMPL_SRCS
  algorithm_impl.cc
  greedy.cc
  ilp.cc
  ilp_exact.cc
  ils.cc
  presolved_ilp.cc
  reductions.cc
  scip.cc
  unfold.cc
)

set(ALGORITHM_IMPL_HDRS
  algorithm_impl.h
  greedy.h
  ilp.h
  ilp_exact.h
  ils.h
  presolved_ilp.h
  reductions.h
  scip.h
  unfold.h
)

if(BMATCHING_USE_KARP_SIPSER)
  list(APPEND ALGORITHM_IMPL_SRCS kss.cc)
  list(APPEND ALGORITHM_IMPL_HDRS kss.h)
endif()

if(BMATCHING_USE_BSUITOR)
  list(APPEND ALGORITHM_IMPL_SRCS bsuitor.cc)
  list(APPEND ALGORITHM_IMPL_HDRS bsuitor.h)
endif()

# OBJECT library ensures all registration statics (REGISTER_IMPL) are linked,
# equivalent to Bazel's alwayslink=1.
add_library(algorithm_impl_lib OBJECT ${ALGORITHM_IMPL_SRCS} ${ALGORITHM_IMPL_HDRS})
target_link_libraries(algorithm_impl_lib PUBLIC
  app_io_cc_proto
  greedy
  ilp
  ils
  reductions_sorted
  bmatching_scip
  bmatching_lib
  graph_reader
  hgr_reader
  systeminfo
  absl::flat_hash_set
  absl::statusor
)

if(BMATCHING_USE_KARP_SIPSER)
  target_link_libraries(algorithm_impl_lib PUBLIC ucar_kss_lib ucar_ksmd_lib)
endif()

if(BMATCHING_USE_BSUITOR)
  target_compile_options(algorithm_impl_lib PRIVATE -fopenmp)
  target_link_options(algorithm_impl_lib PUBLIC -fopenmp)
  target_link_libraries(algorithm_impl_lib PUBLIC pothen_bmatching_lib)
endif()
