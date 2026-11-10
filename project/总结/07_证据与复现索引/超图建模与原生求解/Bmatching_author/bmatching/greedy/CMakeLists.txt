# bmatching/greedy/CMakeLists.txt

add_library(greedy INTERFACE)
target_include_directories(greedy INTERFACE ${CMAKE_SOURCE_DIR})
target_link_libraries(greedy INTERFACE
  bmatching_lib
  indexed_priority_queue
  modifiable_hypergraph_lib
  random_lib
  range_lib
)
