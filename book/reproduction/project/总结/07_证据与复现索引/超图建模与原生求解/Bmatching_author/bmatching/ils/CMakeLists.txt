# bmatching/ils/CMakeLists.txt

add_library(ils INTERFACE)
target_include_directories(ils INTERFACE ${CMAKE_SOURCE_DIR})
target_link_libraries(ils INTERFACE
  bmatching_lib
  indexed_priority_queue
  modifiable_hypergraph_lib
  random_lib
  range_lib
  absl::flat_hash_map
)
