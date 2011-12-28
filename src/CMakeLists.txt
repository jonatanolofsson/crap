include_directories(${CRAP_SOURCE_DIR}/src)
link_directories(${CRAP_BINARY_DIR})

set(CMAKE_CXX_FLAGS "-g -Wall -Wextra")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -rdynamic -ldl")


find_package( Boost COMPONENTS thread )
link_directories ( ${Boost_LIBRARY_DIRS} )
include_directories ( ${Boost_INCLUDE_DIRS} )

include_directories(${YAML_CPP_SOURCE_DIR}/include)
add_executable(crap core.cpp communication.cpp)
target_link_libraries(crap 
    yaml-cpp 
    ${Boost_LIBRARIES})