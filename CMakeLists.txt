cmake_minimum_required(VERSION 2.6)
project(CRAP)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


add_subdirectory(lib)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${YAML_CPP_SOURCE_DIR}/include)
include_directories(${CPPLOT_SOURCE_DIR}/include)
include_directories(${CPPLOT_SOURCE_DIR}/lib/gl2ps)

add_subdirectory(src)
add_subdirectory(modules)
