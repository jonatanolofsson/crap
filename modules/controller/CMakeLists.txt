set(module controller)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lgfortran")

add_library(${module} SHARED ${module}.cpp)
set_target_properties(${module} PROPERTIES PREFIX "")
target_link_libraries(${module}
    ${Boost_LIBRARIES}
    slicot_control
)
