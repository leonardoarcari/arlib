#!/bin/bash

set -e
mkdir -p "$HOME/opt"
OPT="$HOME/opt"

if [ ! -d "$OPT/cmake" ]; then
    echo "==> Installing CMake 3.12..."
    wget https://cmake.org/files/v3.12/cmake-3.12.0-Linux-x86_64.sh
    mkdir -p "$OPT/cmake"
    sh cmake-3.12.0-Linux-x86_64.sh --prefix="$OPT/cmake" --skip-license
fi

if [ true ]; then
    echo "==> Installing Doxygen 1.8.14..."
    wget http://ftp.stack.nl/pub/users/dimitri/doxygen-1.8.14.linux.bin.tar.gz
    mkdir -p "$OPT/doxygen"
    tar vxf "doxygen-1.8.14.linux.bin.tar.gz"
    cp -r "doxygen-1.8.14" "$OPT/doxygen"
fi