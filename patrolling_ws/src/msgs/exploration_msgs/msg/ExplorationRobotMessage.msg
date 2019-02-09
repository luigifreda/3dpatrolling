uint8 kReached   = 0
uint8 kPlanned   = 1  
uint8 kSelected  = 2 
uint8 kAborted   = 3

std_msgs/Header header
uint8 robot_id                   # robot id
uint8 action			 # action
geometry_msgs/Point goal         # the goal
nav_msgs/Path path               # the computed path  
float64 path_cost	         # cost of the planned path: "< 0" means invalid
