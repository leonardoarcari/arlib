cmake_minimum_required( VERSION 3.5 )

# the project name
project( examples )

add_executable(getting_started src/getting_started.cpp)
target_link_libraries(getting_started PUBLIC arlib)

add_executable(softwarex_examples src/softwarex_example.cpp)
target_link_libraries(softwarex_examples PUBLIC arlib)