add_executable(face_reconstruction main.cpp)

target_include_directories(face_reconstruction PUBLIC ${INCLUDE_DIR})

target_link_libraries(face_reconstruction Eigen3::Eigen)