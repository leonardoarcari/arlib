include(CMakeFindDependencyMacro)
find_package(Boost REQUIRED COMPONENTS graph)
include("${CMAKE_CURRENT_LIST_DIR}/arlibTargets.cmake")