set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/modules)

find_package( Eigen3 )
find_package( Boost COMPONENTS thread )
link_directories ( ${Boost_LIBRARY_DIRS} )
include_directories ( ${Boost_INCLUDE_DIRS} )
include_directories ( ${Eigen3_INCLUDE_DIR} )

# Modules with default configuration
set(MODULES
)

# Include directories
set(SUBDIRS
    #~ model
    logic
    controller
    observer
    reality
    sensor_reader
    camera_reader
    baselink
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
