set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/modules)

find_package( Boost COMPONENTS thread )
link_directories ( ${Boost_LIBRARY_DIRS} )
include_directories ( ${Boost_INCLUDE_DIRS} )
include_directories(${YAML_CPP_SOURCE_DIR}/include)

# Modules with default configuration
set(MODULES testmodule comm_sender comm_receiver)
set(CORELIBS yaml-cpp)
foreach(module ${MODULES})
    add_library(${module} SHARED ${module}.cpp)
    target_link_libraries(${module} 
        ${CORELIBS}
        ${Boost_LIBRARIES})
endforeach(module)