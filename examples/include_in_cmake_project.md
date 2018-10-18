# Include ARLib in your CMake project

If you are using CMake as you build system, including **ARLib** is super easy.

First of all clone **ARLib** in your external sources directory, for instance

```bash
$ git clone https://gitlab.com/leonardo_arcari_master_thesis/arlib.git external/arlib
```

##### Dependencies

**ARLib** require a C++17 compiler, CMake (>=3.5) and
[Boost::Graph][boost-graph] (>= 1.65) to be locatable by CMake
`find_package`

##### CMakeLists.txt

To include ARLib to the dependencies of your project, just link `arlib` to your
CMake target.

```cmake
cmake_minimum_required( VERSION 3.5 )
project( my-project )

# Add ARLib directory
add_subdirectory(external/arlib)

add_executable(my_target ...)
target_link_libraries(my_target PUBLIC arlib)
```

##### Compile and Run

From now on, the workflow is the standard CMake one

```bash
$ mkdir -p build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ..
$ make
```

and run your target

```bash
$ build/my_target
```

### Next steps
 - Read the [Tutorial] to write your first alternative-route planner.

 
----------------------------

[Home]
 
[boost-graph]: https://www.boost.org/doc/libs/1_68_0/libs/graph/doc/index.html
[Tutorial]: getting_started.md
[Home]: https://github.com/leonardoarcari/arlib/blob/master/README.md