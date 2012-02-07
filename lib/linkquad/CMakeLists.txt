cmake_minimum_required(VERSION 2.8)
project(linkquad)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
add_library(linkquad serial_communication.cpp link_api/packet.c)
