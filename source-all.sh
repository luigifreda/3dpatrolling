#!/usr/bin/env sh

# N.B.: do not use extend at the first source 

echo "Using patrolling3d_wss *******************************************************************"

THIS_DIR="$(cd "$(dirname "$BASH_SOURCE")"; pwd)"
THIS_DIR=$(readlink -f $THIS_DIR)  # this reads the actual path if a symbolic directory is used
export PATROLLING3D_HOME=$THIS_DIR
echo PATROLLING3D_HOME: $PATROLLING3D_HOME

# load the config environment variables
. $PATROLLING3D_HOME/config.sh

. $PATROLLING3D_HOME/mapping_ws/devel/setup.bash --extend 

. $PATROLLING3D_HOME/patrolling_ws/devel/setup.bash --extend


