set(module baselink)

add_library(${module} SHARED ${module}.cpp)
set_target_properties(${module} PROPERTIES PREFIX "")
target_link_libraries(${module}
    linkquad
    yaml-cpp
)
