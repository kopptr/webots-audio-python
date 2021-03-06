#!/bin/bash

: ${WEBOTS_HOME:?"Please set the WEBOTS_HOME environment variable and try again"}

INCLUDE=include/controller/cpp/webots
SRC=projects/languages/cpp/src
INTERFACE=projects/languages/python/src

echo 'Backing up old files...'
mv ${WEBOTS_HOME}/${INCLUDE}/Robot.hpp ${WEBOTS_HOME}/${INCLUDE}/Robot.hpp.backup
mv ${WEBOTS_HOME}/${SRC}/Robot.cpp ${WEBOTS_HOME}/${SRC}/Robot.cpp.backup
mv ${WEBOTS_HOME}/${SRC}/Makefile ${WEBOTS_HOME}/${SRC}/Makefile.backup
mv ${WEBOTS_HOME}/${INTERFACE}/controller.i ${WEBOTS_HOME}/${INTERFACE}/controller.i.backup

echo 'Copying in new files...'
cp ./${INCLUDE}/Robot.hpp ${WEBOTS_HOME}/${INCLUDE}/Robot.hpp
cp ./${INCLUDE}/Microphone.hpp ${WEBOTS_HOME}/${INCLUDE}/Microphone.hpp
cp ./${INCLUDE}/Speaker.hpp ${WEBOTS_HOME}/${INCLUDE}/Speaker.hpp
cp ./${SRC}/Robot.cpp ${WEBOTS_HOME}/${SRC}/Robot.cpp
cp ./${SRC}/Microphone.cpp ${WEBOTS_HOME}/${SRC}/Microphone.cpp
cp ./${SRC}/Speaker.cpp ${WEBOTS_HOME}/${SRC}/Speaker.cpp
cp ./${SRC}/Makefile ${WEBOTS_HOME}/${SRC}/Makefile
cp ./${INTERFACE}/controller.i ${WEBOTS_HOME}/${INTERFACE}/

echo 'Building the C++ wrapper...'
cd ${WEBOTS_HOME}/projects/languages/cpp/src
make

echo 'Building the Python bindings...'
cd ${WEBOTS_HOME}/projects/languages/python/src
make

echo 'Done'
