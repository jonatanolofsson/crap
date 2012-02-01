set(module logic)
add_library(${module} SHARED ${module}.cpp governor.cpp)
set_target_properties(${module} PROPERTIES PREFIX "")
add_subdirectory(modes)
