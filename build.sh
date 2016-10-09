#!/bin/bash
clear
echo "Building ... "
rm -rf build
mkdir build
cd build
cmake .. > output.log
