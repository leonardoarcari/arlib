# check the minimum version
cmake_minimum_required(VERSION 3.10)

# In case no build type is specified, force Release
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif(NOT CMAKE_BUILD_TYPE)

# ==-----------------------------------------------------------------------== #
#                       ARLib Python bindings
# ==-----------------------------------------------------------------------== #
add_subdirectory(../../ arlib_build EXCLUDE_FROM_ALL)
add_subdirectory(../../external/pybind11 pybind11_build)

set(python_srcs
    src/arlib.cpp
    src/esx.cpp
    src/graph_types.cpp
    src/onepass_plus.cpp
    src/penalty.cpp
)

set(python_headers
    include/esx.hpp
    include/graph_types.hpp
    include/onepass_plus.hpp
    include/penalty.hpp
)

pybind11_add_module(arlib_python ${python_headers} ${python_srcs})
target_include_directories(arlib_python
    PUBLIC
        include
)
target_link_libraries(arlib_python PRIVATE arlib)
# Set the output library name to arlib because that's what setup.py and
# distutils expects.
set_target_properties(arlib_python PROPERTIES OUTPUT_NAME arlib)