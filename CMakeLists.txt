cmake_minimum_required(VERSION 2.6)
project(CRAP)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()

option(CRAP_PLOT ON)
option(CRAP_OUTPUT_DATA ON)
option(EXTENDED_MODEL ON)
configure_file( ${CMAKE_SOURCE_DIR}/include/crap/config.hpp.cmake ${CMAKE_SOURCE_DIR}/include/crap/config.hpp )


set(CMAKE_CXX_FLAGS "-O2 -ggdb -Wall -Wextra -std=c++0x")

# add our macro directory to cmake search path
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(lib)

# add our macro directory to cmake search path
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake)
set(YAMLCPP_STATIC_LIBRARY ON)

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/lib
)
if(CRAP_PLOT)
    find_package(cpplot)
    message(STATUS "sources: ${CPPLOT_INCLUDE_DIR}")
    include_directories(${CPPLOT_INCLUDE_DIR})
    add_subdirectory(interface)
endif(CRAP_PLOT)
include_directories(${linkquad_SOURCE_DIR})

add_subdirectory(src)
add_subdirectory(modules)
