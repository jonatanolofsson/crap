set(mode hover)
add_library(${mode} SHARED ${mode}.cpp)
set_target_properties(${mode} PROPERTIES PREFIX "")
target_link_libraries(${mode}
    ${Boost_LIBRARIES})
