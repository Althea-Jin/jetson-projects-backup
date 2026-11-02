add_library(bspline_opt STATIC
    src/uniform_bspline.cpp
    src/bspline_optimizer.cpp
    src/gradient_descent_optimizer.cpp
)

target_include_directories(bspline_opt PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(bspline_opt PUBLIC
    Eigen3::Eigen
    path_searching
    local_map
)
