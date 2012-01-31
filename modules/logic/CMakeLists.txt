set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/flightmodes)

include_directories ( "." )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Modes with default configuration
set(MODES
    hover
)
foreach(mode ${MODES})
    add_library(${mode} SHARED ${mode}.cpp)
    set_target_properties(${mode} PROPERTIES PREFIX "")
endforeach(mode)
