cmake_minimum_required(VERSION 3.6)

project(face_reconstruction)

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Set C++ flags
set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

add_subdirectory(src)

# Visual Studio properties
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT face_reconstruction)
set_property(TARGET face_reconstruction PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/) 
set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})