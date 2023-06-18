cmake_minimum_required(VERSION 3.6)

project(bfm_manager) # Project name: Base Face Model Manager
# set(HDF5_ROOT /usr/local/hdf5)
# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 17)
if(UNIX)
   add_compile_options(-fPIC)
endif()

add_library(bfm_manager SHARED src/bfm_manager.cpp)
target_compile_features(bfm_manager PRIVATE cxx_std_17)


set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Libs CACHE PATH "Path to lib folder")
set(Eigen3_DIR ${LIBRARY_DIR}/Eigen/cmake CACHE PATH "Path to installed Eigen")
# set(HDF5_ROOT /usr/local/hdf5)
# set(HDF5_LIBRARIES /usr/local/hdf5/lib)
# set(HDF5_INCLUDE_DIRS /usr/local/hdf5/include)
find_package(HDF5 REQUIRED)
find_package(Eigen3 REQUIRED)



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

# OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

target_include_directories(bfm_manager PUBLIC ${OpenCV_INCLUDE_DIRS})

find_package(glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

FIND_PACKAGE( OpenMP REQUIRED)
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
message(STATUS "path to glog cmake information: ${glog_DIR}")
message(STATUS "path to HDF5 lib information: ${HDF5_LIBRARIES}")
message(STATUS "path to HDF5 ROOT information: ${HDF5_ROOT}")
message(STATUS "path to Boost lib information: ${Boost_LIBRARIES}")
message(STATUS "path to opencv dir information: ${OpenCV_DIR}")
message(STATUS "path to opencv include information: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "path to opencv lib information: ${OpenCV_LIBS}")


# Bfm Manager
target_include_directories(bfm_manager PUBLIC ./include ${OpenCV_INCLUDE_DIRS})
target_link_libraries(bfm_manager m ${OpenCV_LIBS} ${HDF5_LIBRARIES} ${HDF5_CXX_LIBRARIES} ${Boost_LIBRARIES} glog boost_filesystem)


###############################################################################################
######################################## Options ##############################################
###############################################################################################


OPTION(GEN_BFM_EXAMPLES "Generate usage examples of Basel Face Moldel manager." ON)
if(GEN_BFM_EXAMPLES)
   add_executable(gen_faces MeshGen/ex_gen_faces.cpp)
   target_link_libraries(gen_faces bfm_manager)
endif()