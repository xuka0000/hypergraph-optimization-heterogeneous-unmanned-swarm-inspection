# bmatching/reductions_sorted/CMakeLists.txt

add_library(reductions_sorted INTERFACE)
target_include_directories(reductions_sorted INTERFACE ${CMAKE_SOURCE_DIR})
target_link_libraries(reductions_sorted INTERFACE
  bmatching_lib
  indexed_priority_queue
  modifiable_hypergraph_lib
  random_lib
  range_lib
  easyloggingpp_lib
)
