
# Install 3dpatrolling

Please, before reading this file, read the main **[README.md](./README.md)** file.

## Prerequisites

We designed and tested the framework under **Ubuntu 14.04**. We recently ported the framework in Ubuntu **16.04** (limited testing was performed here). In any case, it should be easy to compile the framework in other platforms. 


## Quick start

Starting from the root folder of this repo, run the following commands: 
* install V-REP and ROS dependencies
`$ ./install.sh`
* compile all the workspaces 
`$ ./compile-all.sh`
* you can use the following command to clean the workspaces 
`$ ./clean-all.sh `
* source the workspaces by using
`$ source source-all.bash`

Now, you're ready to test the patrolling system or the path planner (see the main **[README.md](./README.md)** file). 

You can find below some manual installation steps. Please, consider that you can find the same commands wrapped for you inside the above installation/compilation scripts. 

## Manually install necessary tools and ROS dependencies

Below you can find some required installation steps. Please, consider that these are wrapped for you inside the installation scripts. 

* install catkin tools following the instructions on this page 
http://catkin-tools.readthedocs.io/en/latest/installing.html

* install V-REP (see the section below)

* install necessary ROS dependencies 
```
$ rosdep install --from-paths mapping_ws/src --ignore-src -r
$ rosdep install --from-paths patrolling_ws/src --ignore-src -r
```

* these are some required packages 
```
sudo apt-get install -y python-rosinstall
sudo apt-get install -y ros-$ROS_DISTRO-octomap-mapping ros-$ROS_DISTRO-octomap-msgs ros-$ROS_DISTRO-octomap-ros ros-$ROS_DISTRO-octomap-server
sudo apt-get install -y ros-$ROS_DISTRO-move-base-msgs 
sudo apt-get install -y ros-$ROS_DISTRO-move-base
sudo apt-get install -y ros-$ROS_DISTRO-tf2-geometry-msgs 
sudo apt-get install -y ros-$ROS_DISTRO-tf2
sudo apt-get install -y ros-$ROS_DISTRO-joy
sudo apt-get install -y ros-$ROS_DISTRO-navigation
sudo apt-get install -y sox
sudo apt-get install -y doxygen
```

**N.B.** every time you have a failure with a [package-name] you can run
`$ rosdep install [package-name]`  this automatically dowloads and installs system dependancies for the package [package-name] at hand.

## Manual V-REP install

Below you can find some required installation steps. Please, consider that these are wrapped for you inside the installation scripts. 

1. Download version 3.2.2 (tested) from here 
http://coppeliarobotics.com/files/V-REP_PRO_EDU_V3_2_2_64_Linux.tar.gz

2. You don't need to compile anything. Just extract the files in your V-REP installation folder and you are ready to execute the main launcher (vrep.sh) from there.

3. Set the environment variable VREP_ROOT_DIR: add in your .bashrc the following line  
`export VREP_ROOT_DIR=<absolute path of your V-REP installation folder (which contains the launcher vrep.sh)>`  
for instance  
`export VREP_ROOT_DIR=/usr/local/V-REP_PRO_EDU_V3_2_2_64_Linux`

4. Once you have updated and compiled the tradr-simulation stack, you have to copy the lib `patrolling_ws/devel/lib/libv_repExtRos.so` in the
installation folder VREP_ROOT_DIR (NOTE: this lib enables V-REP to get and parse track-velocity command messages)

**Note**: at present time, V-REP 3.3 does not work with our framework. Please, use V-REP 3.2.2. We will fix the current issues (UGV model scripts and V-REP 3.3) ASAP.

## Test V-REP installation 

You can test if V-REP is correctly installated by running the following commands: 
`$ cd $VREP_ROOT_DIR`   
`$ sh vrep.sh`   

**Note**: the environment variable should have been set for you by the install script `install.sh` inside  `~/.bashrc`. 

In order to check if the TRADR UGV model is correctly managed, run from the root folder of the 3dpatrolling repo: 
`$ source source-all.bash`   
`$ roslaunch vrep_ugv_simulation vrep_ugv_simulation.launch`    
Once the V-REP main window shows up, press the play button.  
You can then shut down all the nodes by running     
`$ rosrun path_planner kill_vrep_sim`




