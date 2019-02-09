#!/usr/bin/env bash

source devel/setup.bash

catkin build -DCMAKE_BUILD_TYPE=Release "$@"
