#!/bin/bash

if [ $# != 1 ]; then
  echo "No input name!"
  exit 1;
fi

g++ $1.cpp `pkg-config --cflags --libs opencv-2.3.1` -o $1