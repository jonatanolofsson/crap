set(module camera_reader)
add_library(${module} SHARED ${module}.cpp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set_target_properties(${module} PROPERTIES PREFIX "")
target_link_libraries(${module}
    ${Boost_LIBRARIES}
    linkquad
)
