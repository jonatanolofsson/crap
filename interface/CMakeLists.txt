find_package(cpgl REQUIRED)
find_package( Eigen3 )
find_package( Boost COMPONENTS thread )

link_directories ( ${Boost_LIBRARY_DIRS} )
include_directories ( ${Boost_INCLUDE_DIRS} )
include_directories ( ${Eigen3_INCLUDE_DIR} )
include_directories ( ${CMAKE_SOURCE_DIR}/interface )

add_definitions(-DGL_GLEXT_PROTOTYPES)

add_library(spnav_utils spnav_utils.cpp)
target_link_libraries(spnav_utils
    ${Boost_LIBRARIES})

add_executable(3diface 3diface.cpp)
target_link_libraries(3diface
    linkquad
    yaml-cpp
    spnav
    pthread
    X11
    ${CPGL_LIBRARY}
    GL_tools
    spnav_utils
    flyer
)
add_subdirectory(elements)

#~ add_executable(simple_sender simple_sender.cpp)
#~ target_link_libraries(simple_sender
    #~ linkquad
    #~ yaml-cpp
    #~ spnav
    #~ pthread
    #~ X11
    #~ spnav_utils
#~ )
