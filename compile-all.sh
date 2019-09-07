#!/usr/bin/env bash

STARTING_DIR=`pwd`
echo STARTING_DIR: $STARTING_DIR

set -e

source ~/.bashrc 
echo "VREP_ROOT_DIR: $VREP_ROOT_DIR"

source source-all.bash 
cd $PATROLLING3D_HOME


# get into mapping_ws and compile it 
cd mapping_ws 
if [ ! -f src/CMakeLists.txt ]; then
	cd src
	catkin_init_workspace
	cd ..	
fi
catkin build -DCMAKE_BUILD_TYPE=Release
cd $PATROLLING3D_HOME


source source-all.bash 


# get into patrolling_ws and compile it 
cd patrolling_ws 
if [ ! -f src/CMakeLists.txt ]; then
	cd src
	catkin_init_workspace
	cd ..	
fi
catkin_make -DCMAKE_BUILD_TYPE=Release
cd $PATROLLING3D_HOME


# check if the vrep lib is synched
echo ""
LIB_VREP=libv_repExtRos.so
if [ -f patrolling_ws/devel/lib/$LIB_VREP ]; then
	if [ ! -f $VREP_ROOT_DIR/$LIB_VREP ]; then
		echo "copying libv_repExtRos.so in $VREP_ROOT_DIR"
		sudo cp patrolling_ws/devel/lib/$LIB_VREP $VREP_ROOT_DIR		
	fi
	DIFF_LIB=$(diff patrolling_ws/devel/lib/$LIB_VREP $VREP_ROOT_DIR)
	if [ "$DIFF_LIB" != "" ]; then
		echo "copying libv_repExtRos.so in $VREP_ROOT_DIR"
		sudo cp patrolling_ws/devel/lib/$LIB_VREP $VREP_ROOT_DIR	
	else
		echo "$LIB_VREP synched"	
	fi
else
	echo "you still need to compile vrep package"
fi
echo ""

# go back to starting dir 
cd $STARTING_DIR


