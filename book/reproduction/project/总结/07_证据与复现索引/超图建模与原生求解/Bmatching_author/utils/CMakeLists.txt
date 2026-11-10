# utils/CMakeLists.txt

add_library(random_lib INTERFACE)
target_include_directories(random_lib INTERFACE ${CMAKE_SOURCE_DIR})

add_library(range_lib INTERFACE)
target_include_directories(range_lib INTERFACE ${CMAKE_SOURCE_DIR})

add_library(systeminfo systeminfo.cc)
target_include_directories(systeminfo PUBLIC ${CMAKE_SOURCE_DIR})
