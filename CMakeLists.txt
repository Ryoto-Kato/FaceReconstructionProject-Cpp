cmake_minimum_required(VERSION 3.6)

project(face_reconstruction)

set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_subdirectory(BFM_basic_pipeline bfm_manager)
add_subdirectory(../libs/dlib dlib_build)

# Set C++ flags
set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# Visual Studio properties
# set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT face_reconstruction)
# set_property(TARGET face_reconstruction PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)
# set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR})

add_executable(face_reconstruction src/main.cpp)

include_directories(face_reconstruction PUBLIC ${INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})

target_link_libraries(face_reconstruction bfm_manager dlib::dlib Eigen3::Eigen ${OpenCV_LIBS})
