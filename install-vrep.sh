#!/usr/bin/env bash

# ====================================================

function print_blue(){
	printf "\033[34;1m"
	printf "$@ \n"
	printf "\033[0m"
}

# ====================================================

print_blue '================================================'
print_blue 'Installing V-REP'
print_blue '================================================'

set -e


#FILE=V-REP_PRO_EDU_V3_2_2_64_Linux  # not available any more on coppelia website 
FILE=V-REP_PRO_EDU_V3_3_2_64_Linux

DEST_DIR=/usr/local

# =========================================

STARTING_DIR=`pwd`

if [ ! -d temp ]; then
	mkdir temp
fi
cd temp 

# download the file 
if [ ! -f $FILE".tar.gz" ]; then
	wget http://coppeliarobotics.com/files/$FILE".tar.gz"
fi

# extract the file
if [ ! -d $FILE ]; then
	tar xvzf $FILE".tar.gz"
fi

# copy extracted dir in destination dir
if [ ! -d $DEST_DIR/$FILE ]; then
	sudo mv $FILE $DEST_DIR 
fi

# set VREP_ROOT_DIR env var in ~/.bashrc
echo "export VREP_ROOT_DIR=/usr/local/$FILE" >> ~/.bashrc


#rm -R temp 




