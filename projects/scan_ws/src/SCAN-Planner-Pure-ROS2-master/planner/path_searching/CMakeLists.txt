add_library(path_searching STATIC
    src/dyn_a_star.cpp
)

target_include_directories(path_searching PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(path_searching PUBLIC
    Eigen3::Eigen
    local_map
)
