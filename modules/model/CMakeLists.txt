add_library(model SHARED model.cpp)
add_library(testcontrol testcontrol.cpp)
#~ #~
add_executable(testrig testrig.cpp)
target_link_libraries(testcontrol
    cpplot
    slicot_control
    yaml-cpp
    testcontrol
    ${Boost_LIBRARIES})
target_link_libraries(testrig
    model
    cpplot
    testcontrol
    ${Boost_LIBRARIES})
