# tools/CMakeLists.txt

add_executable(binary_to_textproto binary_to_textproto.cc)
target_link_libraries(binary_to_textproto PRIVATE
  app_io_cc_proto
  hypergraph_storage_system
  absl::strings
)
