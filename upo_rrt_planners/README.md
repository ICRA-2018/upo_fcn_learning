# upo_rrt_planners
C++ library of some RRT planners and API for its use with ROS

This is a catkin package of ROS that contains two libraries:

* *upo_rrt_planners*: C++ library that contains the following RRT planners:

	- *simple RRT*: RRT planner in x,y coordinates without reasoning about kinodynamic constraints.
	- *simple RRTstar*: RRT* planner in x,y coordinates without reasoning about kinodynamic constraints.
	- *RRT*: RRT planner with kinodynamic constrains.
	- *RRTstar*: RRT* planner with kinodynamic constrains.


* *upo_rrt_planners_ros*: C++ library that wraps the previous library in order to be used in ROS. 


## Dependences

* Package **navigation_features**.

## Parameters

* **rrt_planner_type**. RRT planner to use (value 1,2,3,4 or 5)
	- 1 RRT. *x,y* state space (no dynamics).
	- 2 RRT*. *x,y* state space (no dynamics).
	- 3 Kinodynamic RRT (*x, y, yaw* state space).
	- 4 Kinodynamic RRT* (*x, y, yaw* state space).
	- 5 Simplified Kinodynamic RRT*. RRT* that does not perform the tree rewiring.
* **rrt_solve_time**. Time in seconds that the RRT* planner is allowed to plan a path. Maximum time to find a path in the case of the RRT.
* **rrt_goal_bias**. probability bias to sample the goal.
* **rrt_max_insertion_dist**. Maximum distance (m) to insert a new node from the nearest node of the sample.
* **rrt_goal_xy_tol**. Tolerance (m) to consider that the goal has been reached in the x,y space.
* **rrt_goal_th_tol**. Tolerance (radians) to consider that the goal has been reached in the angular space.
* **rrt_interpolate_path_dist**. Distance (m) between nodes to perform an interpolation of the resulting path. Use value 0 for no interpolation.

Only for RRT* planner:
* **rrtstar_use_k_nearest**. Boolean to indicate whether to use k-nearest or radius search to find the neighbors in the tree.
* **rrtstar_first_path_biasing**. Boolean to indicate if a sample biasing over the first path found should be performed.
* **rrtstar_first_path_bias**. If *rrtstar_first_path_biasing* is true, this is the bias to sample for the path.
* **rrtstar_first_stddev_bias**. If *rrtstar_first_path_biasing* is true, this is the standard deviation of the gaussian sampling performed over the first path found.


Only for kinodynamic planners:
* **kino_time_step**. Time step (seconds) to propagate the movement of the robot.
* **kino_max_control_steps**. Maximum number of time steps to extend the movement of the robot.
* **kino_linear_acc**. Maximum linear acceleration of the robot (*m/s²*).
* **kino_angular_acc**. Maximum angular acceleration of the robot (*m/s²*).
* **Kino_steering_type**. Extend function to use. Two options.
	- 1 POSQ method.
	- 2 Modified version of the POSQ for more flexible turns.

State Space:
* **rrt_dimensions**. Usually it should be 2 (*x,y* state space) for non-kinodynamic planners, or 3 (*x,y* and *yaw*) for kinodynamic planners.
* **distance_type**. Functions available to calculate the distance between nodes necessary to obtain the nearest neighbor. The options are:
	- 1 Distance calculated as *(x1-x2)+(y1-y2)*.
	- 2 Euclidean distance.
	- 3 If the yaw is in the state space. *w1 * ED + w2 * QD*, where ED is the euclidean distance and QD is the heading changes obtained as *(1 - |qi+1 - qi|)²*. 
* **motion_cost_type**. Function to calculate the cost of moving from one node to the next one. Usually option 2 is employed.
	- 1 Average of the costs. *(Cost1 + Cost2)/2*.
	- 2 Average of the costs and distance between nodes. *(Cost1 + Cost2)/2 * ||node1 - node2||*
	- 3 Average of the costs and exponential of the distance. *(Cost1 + Cost2)/2 * exp(||node1 - node2||)*
	- 4 Sum of the costs. *Cost1 + Cost2*.
* **rrt_size_x**. Size in meters of the *x* dimension. So, the range is [-x, x].
* **rrt_size_y**. Size in meters of the *y* dimension. So, the range is [-y, y].
* **rrt_xy_resolution**. Resolution of the *x,y* space.
* **robot_radius**. Radius of the inscribed circunference of the robot (meters).

Path smoothing:
* **path_smoothing**. Boolean to indicate whether to perform a smoothing of the RRT path obtained.
* **smoothing_samples**. Integer value to indicate the number of samples to include in the sliding window for path smoothing.

Visualization options:
* **visualize_rrt_tree**. Boolean to indicate whether to publish the tree or not as a marker in the topic *~/rrt_tree*. (NOTE: the points of the path obtained are published in the topic *~/rrt_path_points*).
* **visualize_nav_costmap**. Boolean to indicate whether to publish a map of type costmap based on the RRT function cost. The topic is *~/rrt_costmap*.
* **show_rrt_statistics**. If it is enabled (boolean to true), some statistics about the RRT execution are shown on the screen.
* **show_intermediate_states**. If it is enabled (boolean to true), the intermediate states corresponding to the time step between nodes are published as a marker in the topic *~/rrt_path-interpol_points*. Only valid for kinodynamic planners.


The upo_rrt_planners library uses nearest neighbor data structures through the FLANN library. See: M. Muja and D.G. Lowe, "Fast Approximate Nearest Neighbors with Automatic Algorithm Configuration", in International Conference on Computer Vision Theory and Applications (VISAPP'09), 2009. http://people.cs.ubc.ca/~mariusm/index.php/FLANN/FLANN

#### TODO
- [ ] Replace regular pointers by boost smart pointers. 
- [ ] Add new RRT algorithms.
- [ ] Add a plugin to be used as a global planner in the move_base architecture of ROS.


The package is a **work in progress** used in research prototyping. Pull requests and/or issues are highly encouraged.
