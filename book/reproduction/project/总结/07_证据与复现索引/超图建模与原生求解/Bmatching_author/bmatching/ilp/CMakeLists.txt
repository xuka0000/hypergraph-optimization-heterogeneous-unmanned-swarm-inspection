# bmatching/ilp/CMakeLists.txt

add_library(ilp INTERFACE)
target_include_directories(ilp INTERFACE ${CMAKE_SOURCE_DIR})
target_link_libraries(ilp INTERFACE
  bmatching_lib
  indexed_priority_queue
  modifiable_hypergraph_lib
  random_lib
  range_lib
)

if(BMATCHING_USE_GUROBI)
  target_compile_definitions(ilp INTERFACE USE_GUROBI_ENABLED)
  target_link_libraries(ilp INTERFACE Gurobi::Gurobi)
endif()
