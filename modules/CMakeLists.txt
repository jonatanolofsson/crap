set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/modules)

find_package( Boost COMPONENTS thread )
find_package( Eigen3 )
link_directories ( ${Boost_LIBRARY_DIRS} )
include_directories ( ${Boost_INCLUDE_DIRS} )
include_directories ( ${Eigen3_INCLUDE_DIR} )
include_directories(${YAML_CPP_SOURCE_DIR}/include)
include_directories(${MATPLOTPP_SOURCE_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Modules with default configuration
set(MODULES
    #~ testmodule
    comm_sender
    comm_receiver
    observer
    #~ sensor_reader
    logic
)
foreach(module ${MODULES})
    add_library(${module} SHARED ${module}.cpp)
    set_target_properties(${module} PROPERTIES PREFIX "")
    target_link_libraries(${module}
        ${Boost_LIBRARIES})
endforeach(module)

# Controller with slicot library
add_library(controller SHARED controller.cpp)
set_target_properties(controller PROPERTIES PREFIX "")
target_link_libraries(controller
    ${Boost_LIBRARIES}
    slicot_control
)


add_subdirectory("logic")
