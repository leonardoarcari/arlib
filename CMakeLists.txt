# check the minimum version
cmake_minimum_required( VERSION 2.8.12 )

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
# set(CMAKE_CXX_STANDARD 17) can't set. Too old cmake. Crap.
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -static-libgcc -Wl,--hash-style=both,--as-needed -pthread -fopenmp -ftemplate-depth=1100" )
add_compile_options(-g
        -fmessage-length=0
        -c
        -Wall
        -Wextra
        -pedantic
        -Wredundant-decls
        -Wdisabled-optimization
        -Wctor-dtor-privacy
        -Wnon-virtual-dtor
        -Woverloaded-virtual
        -Wsign-promo
        -Wold-style-cast
        -Werror=return-type
        -DLINUX
        -Ofast)

# setting debug flags
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -g3 -O0 -ftemplate-depth=1100")

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
set( APP_NAME "kspwlo")

# Executable 
set(ALGORITHMS_SRC
        src/algorithms/esx.cpp
        src/algorithms/kspwlo.hpp
        src/algorithms/multipass.cpp
        src/algorithms/onepass.cpp
        src/algorithms/onepass_plus.cpp
        src/algorithms/skyline.cpp
        src/algorithms/svp_plus.cpp)
set(MODEL_SRC
        src/model/graph.cpp
        src/model/graph.hpp)
set(TOOLS_SRC
        src/tools/astar.cpp
        src/tools/dijkstra.cpp
        src/tools/tools.hpp)
set(EXPLORATION_SRC
        src/exploration/error_metrics.hpp
        src/exploration/graph_utils.hpp)

set(SOURCE_FILES
        ${ALGORITHMS_SRC}
        ${MODEL_SRC}
        ${TOOLS_SRC}
        ${EXPLORATION_SRC}
        src/main.cpp)

add_executable(${APP_NAME} ${SOURCE_FILES})
target_link_libraries(${APP_NAME} ${MARGOT_HEEL_LIBRARIES})


if (CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set (
      CMAKE_INSTALL_PREFIX "${PROJECT_SOURCE_DIR}"
      CACHE PATH "default install path"
      FORCE )
endif()

install( TARGETS ${APP_NAME} DESTINATION bin )
