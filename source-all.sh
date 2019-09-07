#!/usr/bin/env sh

# N.B.: do not use extend at the first source 

echo "*** Using patrolling3d_wss ***"

THIS_DIR="$(cd "$(dirname "$BASH_SOURCE")"; pwd)"
THIS_DIR=$(readlink -f $THIS_DIR)  # this reads the actual path if a symbolic directory is used
export PATROLLING3D_HOME=$THIS_DIR
echo PATROLLING3D_HOME: $PATROLLING3D_HOME

. ~/.bashrc 

# load the config environment variables
. $PATROLLING3D_HOME/config.sh

setup_files=( \
"$PATROLLING3D_HOME/mapping_ws/devel/setup.bash" \
"$PATROLLING3D_HOME/patrolling_ws/devel/setup.bash" \
)

for setup_file in "${setup_files[@]}"
do
    if [ -f $setup_file ]; then
        . $setup_file --extend
    fi
done


# https://answers.ros.org/question/266313/robot-model-not-showing-in-rviz/
# added for making the robots appear in RVIZ 
export LC_NUMERIC="en_US.UTF-8"
