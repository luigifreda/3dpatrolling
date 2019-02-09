std_msgs/Header header
string name                      # task name; a planning task is a vector of task-segments; a task-segment is composed by a start and a goal (two points). 
int32 task_id
uint8  type                      # NORMAL=0, CYCLIC = 1
geometry_msgs/Point[] waypoints  # array of waypoints
nav_msgs/Path path               # the computed path which joins the waypoints 


