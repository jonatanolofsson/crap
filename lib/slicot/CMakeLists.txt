cmake_minimum_required(VERSION 2.8)
project(SLICOT)
enable_language(Fortran)
find_package(LAPACK REQUIRED)
include_directories(${LAPACK_INCLUDE_DIR})
link_directories(${LAPACK_LIBRARY_DIR})

add_library(slicot_control
    MB04MD.f
    MB05OD.f
    MB05OY.f
    SB02MD.f
    SB02MR.f
    SB02MS.f
    SB02MU.f
    SB02MV.f
    SB02MW.f
    SB02OD.f
    SB02OU.f
    SB02OV.f
    SB02OW.f
    SB02OY.f
)

option(LINK_LAPACK On)
if(LINK_LAPACK)
    target_link_libraries(slicot_control ${LAPACK_LIBRARIES})
endif(LINK_LAPACK)
