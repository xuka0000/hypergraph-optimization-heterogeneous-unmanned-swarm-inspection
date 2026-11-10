# runner/CMakeLists.txt

add_library(hypergraph_storage_system
  hypergraph_storage_system.cc
)
target_link_libraries(hypergraph_storage_system PUBLIC
  app_io_cc_proto
  absl::strings
)

# --- runner binary ---
# runner.cc unconditionally includes ncurses.h, so always find and link it
find_package(Curses REQUIRED)

add_executable(runner runner.cc)
target_link_libraries(runner PRIVATE
  app_io_cc_proto
  easyloggingpp_lib
  absl::flags
  absl::flags_parse
  absl::statusor
  algorithm_impl_lib
  external_graph_bmatching
  ${CURSES_LIBRARIES}
)
target_include_directories(runner PRIVATE ${CURSES_INCLUDE_DIRS})

if(BMATCHING_FREE_MEMORY_CHECK)
  target_compile_definitions(runner PRIVATE HEIHYPBMATCH_MEMORY__REQ_CHECK)
endif()

if(BMATCHING_USE_TCMALLOC)
  target_link_libraries(runner PRIVATE tcmalloc)
endif()

# --- fork_runner binary ---
add_executable(fork_runner fork_runner.cc)
target_link_libraries(fork_runner PRIVATE
  app_io_cc_proto
  easyloggingpp_lib
  absl::flags
  absl::flags_parse
  absl::statusor
  algorithm_impl_lib
  external_graph_bmatching
)

if(BMATCHING_USE_TCMALLOC)
  target_link_libraries(fork_runner PRIVATE tcmalloc)
endif()

# --- generate_experiment_config binary ---
add_executable(generate_experiment_config generate_experiment_config.cc)
target_link_libraries(generate_experiment_config PRIVATE
  hypergraph_storage_system
  app_io_cc_proto
  absl::flags
  absl::flags_parse
  absl::statusor
)
