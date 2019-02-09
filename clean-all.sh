#! /bin/bash
set +e # disable "exit immediately"

# get into mapping_ws
cd mapping_ws 
rm -Rf build/ devel/ logs/
cd ..

# get into patrolling_ws
cd patrolling_ws 
rm -Rf build/ devel/
cd ..

set -e # enable "exit immediately"

