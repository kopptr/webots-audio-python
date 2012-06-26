#!/bin/bash

: ${WEBOTS_HOME:?"Please set the WEBOTS_HOME environment variable and try again"}

INCLUDE=include/controller/cpp/webots
SRC=projects/languages/cpp/src

echo 'Restoring the backups...'
mv ${WEBOTS_HOME}/${INCLUDE}/Robot.hpp.backup ${WEBOTS_HOME}/${INCLUDE}/Robot.hpp
mv ${WEBOTS_HOME}/${SRC}/Robot.cpp.backup ${WEBOTS_HOME}/${SRC}/Robot.cpp

echo 'Removing unneeded files...'
rm ${WEBOTS_HOME}/${INCLUDE}/{Microphone,Speaker}.hpp
rm ${WEBOTS_HOME}/${SRC}/{Microphone,Speaker}.cpp

echo 'Building the original C++ wrapper...'
cd ${WEBOTS_HOME}/projects/languages/cpp/src
make clean
make

echo 'Building the original Python bindings...'
cd ${WEBOTS_HOME}/projects/languages/python/src
make clean
make

echo 'Done'

