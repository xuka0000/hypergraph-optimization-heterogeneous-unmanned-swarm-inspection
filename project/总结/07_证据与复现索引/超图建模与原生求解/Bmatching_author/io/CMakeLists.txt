# io/CMakeLists.txt

add_library(hgr_reader hgr_reader.cc)
target_link_libraries(hgr_reader PUBLIC
  modifiable_hypergraph_lib
  absl::statusor
  absl::strings
  easyloggingpp_lib
)

add_library(graph_reader INTERFACE)
target_include_directories(graph_reader INTERFACE ${CMAKE_SOURCE_DIR})
target_link_libraries(graph_reader INTERFACE
  modifiable_hypergraph_lib
  absl::statusor
  absl::strings
  easyloggingpp_lib
)

add_library(mtx_reader INTERFACE)
target_include_directories(mtx_reader INTERFACE ${CMAKE_SOURCE_DIR})
target_link_libraries(mtx_reader INTERFACE
  modifiable_hypergraph_lib
  absl::strings
)

add_library(hgr_writer hgr_reader.cc)
target_link_libraries(hgr_writer PUBLIC
  modifiable_hypergraph_lib
  absl::strings
)

# --- Binaries ---

add_executable(mtx_to_unihypergraph mtx_to_unihypergraph.cc)
target_link_libraries(mtx_to_unihypergraph PRIVATE hgr_writer mtx_reader)

add_executable(hgr_validator hgr_validator.cc)
target_link_libraries(hgr_validator PRIVATE hgr_reader hgr_writer hypergraph_storage_system)

add_executable(write_mtx write_mtx.cc)
target_link_libraries(write_mtx PRIVATE hgr_reader hgr_writer)

add_executable(hgr_without_weights_to_weighted_and_mtx hgr_without_weights_to_weighted_and_mtx.cc)
target_link_libraries(hgr_without_weights_to_weighted_and_mtx PRIVATE
  hgr_reader hgr_writer app_io_cc_proto
)

add_executable(hgr_import import.cc)
target_link_libraries(hgr_import PRIVATE
  hgr_reader hgr_writer app_io_cc_proto hypergraph_storage_system
  absl::flags absl::flags_parse absl::statusor
)

# --- Tests ---

if(BUILD_TESTING)
  add_executable(hgr_reader_test hgr_reader_test.cc)
  target_link_libraries(hgr_reader_test PRIVATE
    hgr_reader modifiable_hypergraph_lib GTest::gtest_main
  )
  include(GoogleTest)
  gtest_discover_tests(hgr_reader_test)
endif()
