# bmatching/scip/CMakeLists.txt

add_library(bmatching_scip INTERFACE)
target_include_directories(bmatching_scip INTERFACE ${CMAKE_SOURCE_DIR})
target_link_libraries(bmatching_scip INTERFACE
  bmatching_lib
  modifiable_hypergraph_lib
  random_lib
  range_lib
)

if(BMATCHING_USE_SCIP)
  target_compile_definitions(bmatching_scip INTERFACE HEIPHYPBMATCH_USE_SCIP)
  target_link_libraries(bmatching_scip INTERFACE SCIP::SCIP)
endif()
