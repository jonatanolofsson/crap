cmake_minimum_required(VERSION 2.6)
project(CRAP)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()

option(CRAP_PLOT ON)
configure_file( ${CMAKE_SOURCE_DIR}/include/crap/config.hpp.cmake ${CMAKE_SOURCE_DIR}/include/crap/config.hpp )

set(CMAKE_CXX_FLAGS "-O2 -ggdb -Wall -Wextra")

# add our macro directory to cmake search path
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


add_subdirectory(lib)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/lib)
include_directories(${YAML_CPP_SOURCE_DIR}/include)
if(CRAP_PLOT)
    include_directories(${CPPLOT_SOURCE_DIR}/include)
    include_directories(${CPPLOT_SOURCE_DIR}/lib/gl2ps)
endif(CRAP_PLOT)
include_directories(${linkquad_SOURCE_DIR})

add_subdirectory(src)
add_subdirectory(modules)
