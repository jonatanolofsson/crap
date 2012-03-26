set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/flightmodes)

set(MODES
    hover
    landing
    freeflight
)


foreach(mode ${MODES})
    add_subdirectory(${mode})
endforeach(mode)
