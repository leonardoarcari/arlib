# Prepare "Catch" library for other executables
set(CATCH_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/external/catch)
add_library(Catch INTERFACE)
target_include_directories(Catch INTERFACE ${CATCH_INCLUDE_DIR})

set(TEST_SOURCES
        main.cpp
        include/test_graph_utils.cpp
        include/test_onepass_plus.cpp
        include/test_esx.cpp
        include/test_penalty.cpp
        include/test_bidirectional_dijkstra.cpp
        include/test_pruning.cpp
        include/test_reorder_buffer.cpp
        include/test_multi_predecessor_map.cpp
        ${PROJECT_SOURCE_DIR}/external/kspwlo_ref/algorithms/esx.cpp
        ${PROJECT_SOURCE_DIR}/external/kspwlo_ref/algorithms/onepass_plus.cpp
        ${PROJECT_SOURCE_DIR}/external/kspwlo_ref/algorithms/skyline.cpp
        ${PROJECT_SOURCE_DIR}/external/kspwlo_ref/model/graph.cpp
        ${PROJECT_SOURCE_DIR}/external/kspwlo_ref/tools/dijkstra.cpp
        ${PROJECT_SOURCE_DIR}/external/kspwlo_ref/tools/astar.cpp
        ${PROJECT_SOURCE_DIR}/external/kspwlo_ref/exploration/graph_utils.cpp)

add_executable(tests ${TEST_SOURCES})

target_compile_options(tests
    PUBLIC
        -Wall
        -Wextra
        -pedantic
        -ansi
    )

target_include_directories(tests
    PUBLIC
        ${PROJECT_SOURCE_DIR}/test
    PRIVATE
        ${PROJECT_SOURCE_DIR}/external
    )

target_link_libraries(tests
    PUBLIC
        arlib
        Catch
    )