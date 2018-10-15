#!/bin/sh

set -e

git clone https://github.com/doxygen/doxygen.git
mkdir -p $HOME/opt/doxygen
cd doxygen

mkdir build
cd build
cmake -G "Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++-8 -DCMAKE_INSTALL_PREFIX=$HOME/opt/doxygen -Duse_libclang=ON ..
make -j2
make install