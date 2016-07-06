cmake_minimum_required(VERSION 2.6.0)

set(POD_NAME hokuyo_lcm)

project(hokuyo_lcm)

include(cmake/pods.cmake)

add_subdirectory(src/hokuyo)
