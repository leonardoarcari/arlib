cmake_minimum_required( VERSION 3.5 )

# the project name
project( examples )

add_executable(getting_started src/getting_started.cpp)
target_link_libraries(getting_started PUBLIC arlib)