# check the minimum version
cmake_minimum_required( VERSION 3.5 )

# the project name
project( alternative-routing-lib )

# ==-----------------------------------------------------------------------== #
#                         General configure section
# ==-----------------------------------------------------------------------== #
# force the Release build if not already set

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

# ==-----------------------------------------------------------------------== #
#                          Alternative Routing Library
# ==-----------------------------------------------------------------------== #

# Find Boost dependencies
find_package(Boost REQUIRED COMPONENTS graph)

add_subdirectory(include/arlib)
add_subdirectory(src/arlib)

add_library(arlib STATIC ${ARLIB_HEADERS} ${ARLIB_SRC})

target_compile_options(arlib PUBLIC -std=c++17)

target_include_directories(arlib
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    )

target_link_libraries(arlib
    PUBLIC
        Boost::graph
    PRIVATE
        stdc++fs  # std::filesystem
    )

# ==-----------------------------------------------------------------------== #
#                                  Install
# ==-----------------------------------------------------------------------== #
set(arlib_VERSION 0.0.1)

include(CMakePackageConfigHelpers)
write_basic_package_version_file("arlibConfigVersion.cmake"
        VERSION ${arlib_VERSION}
        COMPATIBILITY SameMajorVersion)

# Install library, target and config files
install(TARGETS arlib EXPORT arlibTargets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
    )

install(EXPORT arlibTargets
    FILE arlibTargets.cmake
    NAMESPACE arlib::
    DESTINATION lib/cmake/arlib
    )

install(FILES
        "${PROJECT_SOURCE_DIR}/cmake/arlibConfig.cmake"
        "${PROJECT_BINARY_DIR}/arlibConfigVersion.cmake"
    DESTINATION
        lib/cmake/arlib)

# Install library headers
install(DIRECTORY include/arlib DESTINATION include
        FILES_MATCHING PATTERN "*.hpp")

# ==-----------------------------------------------------------------------== #
#                                Unit Tests
# ==-----------------------------------------------------------------------== #
add_subdirectory(test)
enable_testing(true)
add_test(NAME tests COMMAND tests)

# ==-----------------------------------------------------------------------== #
#                                   Other
# ==-----------------------------------------------------------------------== #

# Documentation
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/doc")

# Examples
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/examples")