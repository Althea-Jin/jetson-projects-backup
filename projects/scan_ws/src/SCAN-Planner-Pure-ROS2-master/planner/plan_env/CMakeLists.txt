add_library(local_map STATIC
    src/grid_map.cpp
)

target_include_directories(local_map PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(local_map PUBLIC
    Eigen3::Eigen
)
