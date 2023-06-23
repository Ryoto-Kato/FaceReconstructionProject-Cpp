cmake_minimum_required(VERSION 3.6)

set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE)

project(face_reconstruction) # Project name: Base Face Model Manager

set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs CACHE PATH "Path to lib folder")
set(Eigen3_DIR ${LIBRARY_DIR}/Eigen/cmake CACHE PATH "Path to installed Eigen")
add_subdirectory(${LIBRARY_DIR}/dlib dlib_build)

option(MINF "Use" ON)

# Use pre-compiled libraries on Windows
if(WIN32)
    if(CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(FreeImage_INCLUDE_DIR ${LIBRARY_DIR}/FreeImage-3.18.0/Dist/x64/ CACHE PATH "Path to FreeImage header file")
        set(FreeImage_LIBRARY_DIR ${LIBRARY_DIR}/FreeImage-3.18.0/Dist/x64/ CACHE PATH "Path to FreeImage .lib/.dll folder")
    elseif(CMAKE_SIZEOF_VOID_P EQUAL 4)
        set(FreeImage_INCLUDE_DIR ${LIBRARY_DIR}/FreeImage-3.18.0/Dist/x32/ CACHE PATH "Path to FreeImage header file")
        set(FreeImage_LIBRARY_DIR ${LIBRARY_DIR}/FreeImage-3.18.0/Dist/x32/ CACHE PATH "Path to FreeImage .lib/.dll folder")
    endif()
endif(WIN32)

# set(HDF5_ROOT /usr/local/hdf5)
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
if(UNIX)
   add_compile_options(-fPIC)
endif()

# add_library(pose_estimation SHARED ../src/pose_estimation.cpp)
# set(HEADERS
#       ./include/Eigen.h
#       ./include/BFM.h
#       ./include/FacialLandmarkExtractor.h
#       ./include/ProcrustesAligner.h
#       ./include/RGBImage.h
#       ./include/SimpleMesh.h
#       ./include/bfm_manager.h
#       ./include/constant.h
#       ./include/data.hpp
#       ./include/random.hpp
#       ./include/transform.hpp
#       ./include/type_utils.hpp
# )

set(SOURCES
   ./src/bfm_manager.cpp
   ./src/FreeImageHelper.cpp
)

add_library(face_reconstruction SHARED ${SOURCES})
target_compile_features(face_reconstruction PRIVATE cxx_std_17)

# set(HDF5_ROOT /usr/local/hdf5)
# set(HDF5_LIBRARIES /usr/local/hdf5/lib)
# set(HDF5_INCLUDE_DIRS /usr/local/hdf5/include)
find_package(HDF5 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(flann REQUIRED)
add_definitions("-D_DISABLE_EXTENDED_ALIGNED_STORAGE")

if (MSVC)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj")
endif()
###############################################################################################
######################################## Dedendenflanncy ###########################################
###############################################################################################

if(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wall -Wextra -Wundef -pedantic")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpmath=sse -funroll-loops -Wno-reorder")
endif()

# Eigen3
if(UNIX)
   include_directories("/usr/include/eigen3")
endif()

# Hdf5
# Explicitely list required HDF5 components
# find_package(HDF5 COMPONENTS C CXX HL REQUIRED)
include_directories(${HDF5_INCLUDE_DIRS})
# set(HDF5_CXX_LIBRARIES /home/ryotok/library/c++_lib/CMake-hdf5-1.12.2/hdf5-1.12.2/hdf5/lib)

#freeimage
link_directories(${FreeImage_LIBRARY_DIR})

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

target_include_directories(face_reconstruction PUBLIC ${OpenCV_INCLUDE_DIRS})

find_package(glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

FIND_PACKAGE(OpenMP REQUIRED)
if(OPENMP_FOUND)
   message("OPENMP FOUND")
   set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
   set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


find_package(Boost REQUIRED system filesystem program_options)
include_directories(${Boost_INCLUDE_DIRS})

if(UNIX)
    set(LIBS 
        /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.65.1
        /usr/lib/x86_64-linux-gnu/libboost_system.so.1.65.1
    )
endif()

message(STATUS "path to Eiegn: ${Eigen3_DIR}")
message(STATUS "path to ceres cmake information: ${Ceres_DIR}")
message(STATUS "path to flann cmake information = ${flann_DIR}")
message(STATUS "path to freeimage include directory: ${FreeImage_INCLUDE_DIR}")
message(STATUS "path to glog cmake information: ${glog_DIR}")
message(STATUS "path to HDF5 lib information: ${HDF5_LIBRARIES}")
message(STATUS "path to HDF5 ROOT information: ${HDF5_ROOT}")
message(STATUS "path to Boost lib information: ${Boost_LIBRARIES}")
message(STATUS "path to opencv dir information: ${OpenCV_DIR}")
message(STATUS "path to opencv include information: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "path to opencv lib information: ${OpenCV_LIBS}")


link_directories(${FreeImage_LIBRARY_DIR})
# Bfm Manager
target_include_directories(face_reconstruction PUBLIC ./include ${OpenCV_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${FreeImage_INCLUDE_DIR} ${Ceres_INCLUDE_DIR} ${glog_INCLUDE_DIR} ${Flann_INCLUDE_DIR})
target_link_libraries(face_reconstruction m ${OpenCV_LIBS} ${HDF5_LIBRARIES} ${HDF5_CXX_LIBRARIES} ${Boost_LIBRARIES} glog::glog boost_filesystem dlib::dlib Eigen3::Eigen freeimage Ceres::ceres flann::flann)

###############################################################################################
######################################## Options ##############################################
###############################################################################################

OPTION(BFM_INIT "Generate usage examples to get BFM components." ON)
if(BFM_INIT)
   add_executable(bfm_init examples/bfm_init.cpp)
   target_link_libraries(bfm_init face_reconstruction)
endif()

OPTION(POSE_ESTIMATION "Pose estimation given dlib landmarks and bfm landmarks and write both .ply." ON)
if(POSE_ESTIMATION)
   add_executable(pose_estimation examples/pose_estimation.cpp)
   target_link_libraries(pose_estimation face_reconstruction)
endif()

OPTION(PARAMS_ESTIMATION "Parameter estimation and write result in .ply." ON)
if(PARAMS_ESTIMATION)
   add_executable(params_estimation examples/parameter_estimation.cpp)
   target_link_libraries(params_estimation face_reconstruction)
endif()

OPTION(BUNNY_PROCRUSTES "Align stanford bunny." ON)
if(BUNNY_PROCRUSTES)
   add_executable(bunny_procrustes examples/bunny_procrustes.cpp)
   target_link_libraries(bunny_procrustes face_reconstruction)
endif()

OPTION(PROCRUSTES_PE "Inplement pose estimation by Procrustes." ON)
if(PROCRUSTES_PE)
   add_executable(procrustes_pe examples/procrustes_pe.cpp)
   target_link_libraries(procrustes_pe face_reconstruction)
endif()

OPTION(ICP_PE "Inplement pose estimation by Rigid ICP." ON)
if(ICP_PE)
   add_executable(icp_pe examples/icp_pe.cpp)
   target_link_libraries(icp_pe face_reconstruction)
endif()

OPTION(GEN_BFM_EXAMPLES "Generate usage examples of Basel Face Moldel manager." ON)
if(GEN_BFM_EXAMPLES)
   add_executable(gen_faces examples/ex_gen_faces.cpp)
   target_link_libraries(gen_faces face_reconstruction)
endif()

OPTION(CERES_EXAMPLE "Ceres optimizer example from exercise 5." ON)
if(CERES_EXAMPLE)
   add_executable(ceres_example examples/example_ceres_optimizer.cpp ./src/FreeImageHelper.cpp)
   target_link_libraries(ceres_example face_reconstruction)
endif()

if(WIN32)
    # Visual Studio properties
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT exercise_1)
    set_property(TARGET exercise_1 PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/)

    # On Windows copy dll to executable folder
    add_custom_command(TARGET exercise_1 POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${FreeImage_LIBRARY_DIR}/FreeImage.dll"
            ${CMAKE_BINARY_DIR})
endif(WIN32)