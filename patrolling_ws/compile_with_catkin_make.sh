#!/usr/bin/env bash

source devel/setup.bash

catkin_make -DCMAKE_BUILD_TYPE=Release "$@" 
