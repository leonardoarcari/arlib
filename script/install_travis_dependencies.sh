#!/bin/bash

set -e
mkdir -p "$HOME/opt"
OPT="$HOME/opt"

if [ ! -f "$OPT/bin/cmake" ]; then
    echo "==> Installing CMake 3.12..."
    wget https://cmake.org/files/v3.12/cmake-3.12.0-Linux-x86_64.sh
    mkdir -p "$OPT/cmake"
    sh cmake-3.12.0-Linux-x86_64.sh --prefix="$OPT/cmake" --skip-license
    ln -s "$OPT/cmake/bin/cmake" "$OPT/bin/cmake"
fi

if [ ! -f "$OPT/bin/doxygen" ]; then
    echo "==> Installing Doxygen 1.8.14..."
    git clone https://github.com/doxygen/doxygen.git
    cd doxygen
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_COMPILER=g++-8 -DCMAKE_INSTALL_PREFIX="$OPT" ..
    make -j2
    make install
fi