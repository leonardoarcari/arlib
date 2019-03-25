#!/bin/bash

set -e
mkdir -p "$HOME/opt"
OPT="$HOME/opt"

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