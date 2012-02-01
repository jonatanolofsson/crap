set(module observer)
add_library(${module} SHARED ${module}.cpp)
set_target_properties(${module} PROPERTIES PREFIX "")
target_link_libraries(${module}
    ${Boost_LIBRARIES})
