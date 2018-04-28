# check the minimum version
cmake_minimum_required( VERSION 3.10 )

# the project name
project( kspwlo )

################################
#### General configure section
################################

# force the Release build if not already set
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

# setting common c++ flags
set(CMAKE_CXX_STANDARD 17 )

add_compile_options(-Wall
        -Wextra
        -pedantic
        -ansi)

# setting debug flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -Og -ftemplate-depth=1100")

# setting release with debug info flags
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -march=native -mtune=native -g3 -O2 -ftemplate-depth=1100")

# setting release flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -mtune=native -O3 -ftemplate-depth=1100")

# check if we need to activate the validation of the results
option( VALIDATION "Validate the results on the error" OFF)
if (VALIDATION)
        set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DVALIDATION_ERROR_MARGOT")
endif (VALIDATION)

###############################################
##             EXTERNAL LIB                  ##
###############################################

# add the path to the modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/margot_heel_if/cmake")

# find the required packages
find_package(MARGOT_HEEL)

# add the required include directories
include_directories(${MARGOT_HEEL_INCLUDES})
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/external")

# Boost graph
find_package(Boost REQUIRED COMPONENTS graph program_options)

###############################################
##                CATCH V2                   ##
###############################################
# Prepare "Catch" library for other executables
set(CATCH_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/catch)
add_library(Catch INTERFACE)
target_include_directories(Catch INTERFACE ${CATCH_INCLUDE_DIR})


###############################################
##             CONFIG FILES                  ##
###############################################
#used to generate the .clang_complete file that is used in vim/atom plugins for autocomplete. 
#can be safely removed if not needed.
#configure_file(
#  "${CMAKE_CURRENT_SOURCE_DIR}/config/clang_autocomplete.in"
#  "${PROJECT_SOURCE_DIR}/.clang_complete"
#  @ONLY
#  )
set (CMAKE_EXPORT_COMPILE_COMMANDS 1)

###############################################
##             BUILD SECTION                 ##
###############################################

# Set the application name
set( KSPWLO_REF "kspwlo_ref")

# Executable 
set(KSPWLO_REF_ALGORITHMS_SRC
        external/kspwlo_ref/algorithms/esx.cpp
        external/kspwlo_ref/algorithms/kspwlo.hpp
        external/kspwlo_ref/algorithms/multipass.cpp
        external/kspwlo_ref/algorithms/onepass.cpp
        external/kspwlo_ref/algorithms/onepass_plus.cpp
        external/kspwlo_ref/algorithms/skyline.cpp
        external/kspwlo_ref/algorithms/svp_plus.cpp)
set(KSPWLO_REF_MODEL_SRC
        external/kspwlo_ref/model/graph.cpp
        external/kspwlo_ref/model/graph.hpp)
set(KSPWLO_REF_TOOLS_SRC
        external/kspwlo_ref/tools/astar.cpp
        external/kspwlo_ref/tools/dijkstra.cpp
        external/kspwlo_ref/tools/tools.hpp
        external/kspwlo_ref/exploration/graph_utils.cpp)

set(KSPWLO_REF_SOURCE_FILES
        ${KSPWLO_REF_ALGORITHMS_SRC}
        ${KSPWLO_REF_MODEL_SRC}
        ${KSPWLO_REF_TOOLS_SRC}
        ${KSPWLO_REF_EXPLORATION_SRC}
        external/kspwlo_ref/main.cpp)

add_executable(${KSPWLO_REF} ${KSPWLO_REF_SOURCE_FILES})

find_package(OpenMP)
if (OPENMP_FOUND)
        target_compile_options(${KSPWLO_REF} PUBLIC -fopenmp)
endif(OPENMP_FOUND)
target_link_libraries(${KSPWLO_REF} ${MARGOT_HEEL_LIBRARIES})
target_link_libraries(${KSPWLO_REF} omp)

# kSPwLO algorithm executable
set(KSPWLO "kspwlo")

set(OP_PLUS_SRC
        dse_apps/driver.cpp)
add_executable(${KSPWLO} ${OP_PLUS_SRC})

# Compiling
option(PERF_PROFILING "Enable compilation flags for Linux Perf profiling" OFF)
if(PERF_PROFILING)
        target_compile_options(${KSPWLO} PUBLIC "-g" "-fno-omit-frame-pointer")
endif(PERF_PROFILING)

# Linking
target_link_libraries(${KSPWLO} ${MARGOT_HEEL_LIBRARIES})
target_link_libraries(${KSPWLO} stdc++fs)
target_link_libraries(${KSPWLO} Boost::graph Boost::program_options)


# Testing
# Make test executable
set(TEST_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/test/main.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/test/include/test_graph_utils.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/test/include/test_onepass_plus.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/test/include/test_esx.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/kspwlo/graph_utils.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/external/kspwlo_ref/algorithms/esx.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/external/kspwlo_ref/algorithms/onepass_plus.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/external/kspwlo_ref/algorithms/skyline.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/external/kspwlo_ref/model/graph.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/external/kspwlo_ref/tools/dijkstra.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/external/kspwlo_ref/tools/astar.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/external/kspwlo_ref/exploration/graph_utils.cpp)
add_executable(tests ${TEST_SOURCES})
target_include_directories(tests PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/test)
target_link_libraries(tests Catch)
target_link_libraries(tests Boost::graph stdc++fs)

enable_testing(true)
add_test(NAME tests COMMAND tests)


if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set (
      CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}"
      CACHE PATH "default install path"
      FORCE )
endif()

install( TARGETS ${APP_NAME} DESTINATION bin )

# Documentation
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/doc")

