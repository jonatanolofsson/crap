include_directories(${CMAKEDEMO_SOURCE_DIR}/src)
link_directories(${CMAKEDEMO_BINARY_DIR})

set(CMAKE_CXX_FLAGS "-g -Wall -Wextra")

include_directories(${YAML_CPP_SOURCE_DIR}/include)
add_executable(crap core.cpp)
target_link_libraries(crap yaml-cpp)