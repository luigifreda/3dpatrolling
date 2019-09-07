# Screen 

A brief list of the main screen commands 

## Check the active sessions 

`$ screen -list`


## Connect to an active session 

`$ screen -R <session_name>`


## Once connected to a session 

* detach from that session: first, press `CTRL+A`, then `D`

* kill that session: first, press `CTRL+A`, then `K`


## Kill all the active screen sessions 

`$ killall screen; screen -wipe`

Indeed, the command `screen -wipe` is used to clean the screen list 


