set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/modules)

find_package( Boost COMPONENTS thread )
find_package( Eigen3 )
link_directories ( ${Boost_LIBRARY_DIRS} )
include_directories ( ${Boost_INCLUDE_DIRS} )
include_directories ( ${Eigen3_INCLUDE_DIR} )
include_directories(${YAML_CPP_SOURCE_DIR}/include)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Modules with default configuration
set(MODULES
    comm_sender
    comm_receiver
)

# Include directories
set(SUBDIRS
    logic
    controller
    observer
    sensor_reader
)


foreach(module ${MODULES})
    add_library(${module} SHARED ${module}.cpp)
    set_target_properties(${module} PROPERTIES PREFIX "")
    target_link_libraries(${module}
        ${Boost_LIBRARIES})
endforeach(module)

foreach(dir ${SUBDIRS})
add_subdirectory(${dir})
endforeach(dir)
