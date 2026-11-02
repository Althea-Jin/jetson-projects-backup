add_library(scan_planner STATIC
    src/planner_interface.cpp
)

target_include_directories(scan_planner PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/plan_manage
    ${CMAKE_CURRENT_SOURCE_DIR}/../include
)
target_link_libraries(scan_planner PUBLIC
    Eigen3::Eigen
    bspline_opt
    path_searching
    local_map
)
