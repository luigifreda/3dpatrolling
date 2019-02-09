# PATH PLANNING STACK

=======================================
## Node: mapping
---------------------------------------

Description: 
aggregates point clouds from laser scanner and remove outdated dynamic obstacles from map

Subscribed topics:
	/dynamic_point_cloud: 3d point cloud from laser scanner

Published topics:
	/dynjoinpcl: aggregated point cloud
	/normals_marker: debug normals estimation

Parameters:
	DynamicJoinPcl:
		global_frame: /map
		laser_frame: /laser
		leaf_size: 0.035 (downsampling leaf size)
		num_subdivisions: 50 (number of subdivisions. higher number -> higher accuracy. beware: too much high number -> out of memory)
	NormalEstimationPcl:
		flatness_curvature_threshold: 0.2
		kernel_type: 0
		laser_frame: /laser
		num_threads: 4
		radius: 0.2
		smoothing: 1.0

=======================================
## Node: traversability
---------------------------------------

Description: 
provides traversability analysis

Subscribed topics:
	/dynjoinpcl: 3d map

Published topics:
	/clustered_pcl/wall: subset of map relative to non-traversable parts (walls, obstacles, ...)
	/clustered_pcl/no_wall: subset of map relative to traversable parts (ground, ramp, stairs)
	/clustered_pcl/segmented: ?
	/normals_pcl: point cloud normals
	/normals_markers: point cloud normals marker
	/trav/label: points labeled with semantic information (a priori cost based on the type of traversable part)
	/trav/density: points labeled with local point cloud density
	/trav/roughness: points labeled with local point cloud roughness
	/trav/clearance: points labeled with information about distance to closest obstacle
	/trav/traversability: costmap summarizing the 4 cost components (label, density, roughness, clearance)

Parameters:
	ClusteringPcl:
		max_superable_height: 0.2 (maximum height of a surmountable obstacle)
		normal_clustering_thres: 0.1 (?)
		normal_radius_search: 0.3 (normal estimation radius search)
		pcl_cluster_tolerance: 0.3 (?)

=======================================
## Node: pathPlanner
---------------------------------------

Description: 
path planning facility

Subscribed topics:
	/trav/traversability: traversability costmap
	/clustered_pcl/wall: (don't know why this guy is here)
	/goal_topic: geometric goal location

Published topics:
	/robot_path: geometric path towards goal (if any)

