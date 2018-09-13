# check the minimum version
cmake_minimum_required( VERSION 3.5 )

# the project name
project( kspwlo )

# ==-----------------------------------------------------------------------== #
#                         General configure section
# ==-----------------------------------------------------------------------== #
# force the Release build if not already set
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

# setting common c++ flags
# set(CMAKE_CXX_STANDARD 17 )

add_compile_options(-Wall
    -Wextra
    -pedantic
    -ansi
    -std=c++17)

# setting debug flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -ftemplate-depth=1100")

# setting release with debug info flags
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -march=native -mtune=native -g3 -O2 -ftemplate-depth=1100")

# setting release flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -mtune=native -O3 -ftemplate-depth=1100")


# ==-----------------------------------------------------------------------== #
#                              kSPwLO Library
# ==-----------------------------------------------------------------------== #

# Find Boost dependencies
find_package(Boost REQUIRED COMPONENTS graph program_options)

set(KSPWLO_LIB_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/include/kspwlo/reorder_buffer.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/kspwlo/impl/esx_impl.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/kspwlo/graph_utils.cpp)

add_library(kspwlo_lib STATIC ${KSPWLO_LIB_SRC}) # Create target
target_include_directories(kspwlo_lib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/src>
        $<INSTALL_INTERFACE:include>
    )

target_link_libraries(kspwlo_lib
    PUBLIC
        Boost::graph
    PRIVATE
        Boost::program_options
        stdc++fs  # std::filesystem
    )

install(TARGETS kspwlo_lib EXPORT kspwloTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    )

install(EXPORT kspwloTargets
    FILE kspwloTargets.cmake
    NAMESPACE kspwlo::
    DESTINATION lib/cmake/kspwlo
    )

# ==-----------------------------------------------------------------------== #
#                                Unit Tests
# ==-----------------------------------------------------------------------== #

# Prepare "Catch" library for other executables
set(CATCH_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/catch)
add_library(Catch INTERFACE)
target_include_directories(Catch INTERFACE ${CATCH_INCLUDE_DIR})

set(TEST_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/test/main.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/test/include/test_graph_utils.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/test/include/test_onepass_plus.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/test/include/test_esx.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/test/include/test_penalty.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/test/include/test_bidirectional_dijkstra.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/test/include/test_pruning.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/test/include/test_reorder_buffer.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/external/kspwlo_ref/algorithms/esx.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/external/kspwlo_ref/algorithms/onepass_plus.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/external/kspwlo_ref/algorithms/skyline.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/external/kspwlo_ref/model/graph.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/external/kspwlo_ref/tools/dijkstra.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/external/kspwlo_ref/tools/astar.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/external/kspwlo_ref/exploration/graph_utils.cpp)

add_executable(tests ${TEST_SOURCES})
target_include_directories(tests
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/test
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/external
    )

target_link_libraries(tests
    PUBLIC
        kspwlo_lib
        Catch
    )

enable_testing(true)
add_test(NAME tests COMMAND tests)

# ==-----------------------------------------------------------------------== #
#                                Other
# ==-----------------------------------------------------------------------== #

# Documentation
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/doc")

