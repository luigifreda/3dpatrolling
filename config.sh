#!/usr/bin/env bash

# here you can load some common env variables 

MR3D_USE_SCREEN=1  # 1 use screen for running ros commands in the scripts; 
                   # 0 use xterm 

########################################################################################################

if [[ -z "${PATROLLING3D_HOME}" ]]; then
    echo "ERROR: missing env var PATROLLING3D_HOME"
    echo "please, source source-all.bash in the main patrolling3d folder"
    exit 
fi

########################################################################################################

SOURCE_FILE=$PATROLLING3D_HOME/source-all.bash

function open_screen(){
session_name=$1
command_string=${@:2}
#screen -dmS session_name -Lc file_init  
screen -dmS $session_name -L  
screen -S $session_name -X stuff $"source $SOURCE_FILE; $command_string \n"   
}

function open_xterm(){
session_name=$1
command_string=${@:2}
xterm -T $session_name $XTERM_OPTIONS -e "$command_string; bash" &    
}
# NOTE: you can use the following command to get the xterm window live if the app terminates or crashes
# xterm $XTERM_OPTIONS -e "<you_command>; bash" &

# usage: 
# open_term "NAME_SESSION" "COMMAND_STRING"
# N.B.: "NAME_SESSION" must be a single word without spaces 
function open_term(){
    if [ $MR3D_USE_SCREEN -eq 1 ]; then
        open_screen $@
    else
        open_xterm $@
    fi
}

########################################################################################################
