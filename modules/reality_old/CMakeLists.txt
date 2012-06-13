set(module reality)
add_library(${module} SHARED ${module}.cpp ../model/model.cpp)
set_target_properties(${module} PROPERTIES PREFIX "")
target_link_libraries(${module}
    ${Boost_LIBRARIES})
