# ds/CMakeLists.txt

add_library(modifiable_hypergraph_lib modifiable_hypergraph.cc)
target_link_libraries(modifiable_hypergraph_lib PUBLIC range_lib)

add_library(bmatching_lib bmatching.cc)
target_link_libraries(bmatching_lib PUBLIC
  modifiable_hypergraph_lib
  random_lib
  range_lib
  easyloggingpp_lib
  wide_integer_lib
)

add_library(indexed_priority_queue INTERFACE)
target_include_directories(indexed_priority_queue INTERFACE ${CMAKE_SOURCE_DIR})

if(BUILD_TESTING)
  add_executable(modifiable_hypergraph_test modifiable_hypergraph_test.cc)
  target_link_libraries(modifiable_hypergraph_test PRIVATE
    bmatching_lib
    modifiable_hypergraph_lib
    reductions_sorted
    hgr_reader
    GTest::gtest_main
  )
  include(GoogleTest)
  gtest_discover_tests(modifiable_hypergraph_test)
endif()
