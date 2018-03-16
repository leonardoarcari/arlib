#!/bin/bash

# Per application definitions
CONF_NAME="kspwlo.conf"

# Compute paths
WORKING_DIR=`pwd`
APPLICATION_ROOT=$WORKING_DIR
MARGOT_ROOT=$WORKING_DIR/../core2/

# Build flags
CLANG="-DCMAKE_CXX_COMPILER=clang++"

# First reset margot heel if
rm -rf $APPLICATION_ROOT/margot_heel_if/build
rm $APPLICATION_ROOT/margot_heel_if/config/*.conf

# Configure the application in autotuning mode
cp $APPLICATION_ROOT/dse_apps/$CONF_NAME $APPLICATION_ROOT/margot_heel_if/config/ || exit -1

# Compile margot heel interface
mkdir -p $APPLICATION_ROOT/margot_heel_if/build || exit -1
cd $APPLICATION_ROOT/margot_heel_if/build
cmake $CLANG -DCMAKE_BUILD_TYPE=Release -DMARGOT_CONF_FILE=$CONF_NAME .. || exit -1
make -j4 || exit -1

# Compile application itself
mkdir -p $APPLICATION_ROOT/build || exit -1
cd $APPLICATION_ROOT/build
cmake $CLANG -DCMAKE_BUILD_TYPE=Release .. || exit -1
make -j4 || exit -1

# Restore working directory
cd $WORKING_DIR

