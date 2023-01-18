################################# For System Setup ##################################
1. In required resources are
	a. V-REP (CoppeliaSim) Software (https://www.coppeliarobotics.com/)
	b. Python3 with IDE of user choice

2. To connect V-REP (CoppeliaSim) with python, python remote API Binding are setted up by following the document "Simulation Challenge - Python Scripting.pdf" in repository
or link "https://www.york.ac.uk/media/physics-engineering-technology/study/engineering-pre-arrival-information/Simulation%20Challenge%20-%20Python%20Scripting.pdf"
######################################################################################



####################### Swarm Aggrgation using AIIS Algorithm ######################

1.Make sure to have CoppeliaSim running, with followig scene loaded: 
Scenes/Aggregation_e_puck_Swarm.ttt

2.The client side script "Aggregation_Swarm_Robots_Avoiding_Obstacles.py" depends on following python scripts:
sim.py, simConst.py, robot_epuck.py, utlis.py, robot_generation.py, regulation_mechanism.py, and the remote API library available
in CoppeliaRobotics\CoppeliaSimPro\programming\remoteApiBindings\lib\lib\Windows.

3. To load the robots model of coppeliasim in python run the "robot_generation.py" and input number of robots that user placed in Coppeliasim scene. 
This program write scene properties in a file with the name of "robot_file", By doing so, there is no need to load scene for each experiment of aggregation.

4. Now run the "Aggregation_Swarm_Robots_Avoiding_Obstacles.py" file. First of all robots models from file "robot_file" are loaded.
Enter the total execution time for experiment and diffusion rate value which is 0.08.

5.There are some parameters which can be changed according to your requiremwnt in script.
Parameters:
	Execution time of experiment = execu_time = 600 sec
	Maximum Sensor range = sensor_range = 0.25
	Minimum safe distance for sensor = min_safe_dist = 0.1
	Maximum safe distance for sensor = max_safe_dist = 0.2
	Distance from object at which object avoidance activates = max_avoid_dist D_Safe = 0.2
	minimum number of required neighboring robots = min_req_nei R_f = 2
	Diffusion rate of danger in Dc regulation = diff_rate = 0.08 
	expe_total_inflam = number of robots * x (recommended x = 3, that means ideally a robot must have 3 neighbors)
######################################################################################



####################### Swarm Flocking using AIIS Algorithm ######################

1.Make sure to have CoppeliaSim running, with followig scene loaded: 
Scenes/Flocking_towards_light_e_puck_Swarm.ttt

2.The client side script "Flocking_Swarm_Towrds_Light_Avoiding_Obstacles.py" depends on following python scripts:
sim.py, simConst.py, robot_epuck.py, utlis.py, robot_generation.py, and the remote API library available
in CoppeliaRobotics\CoppeliaSimPro\programming\remoteApiBindings\lib\lib\Windows.

3. To load the robots model of coppeliasim in python run the "robot_generation.py" and input number of robots that user placed in Coppeliasim scene. 
This program write scene properties in a file with the name of "robot_file", By doing so, there is no need to load scene for each experiment of Flocking.

4. Now run the "Flocking_Swarm_Towrds_Light_Avoiding_Obstacles.py" file and enter the total execution time for experiment.

5.There are some parameters which can be changed according to your requiremwnt in script.
Parameters:
	Execution time of experiment = execu_time = 600 sec
	Maximum Sensor range = sensor_range = 0.25
	Minimum safe distance for sensor = min_safe_dist = 0.11
	Maximum safe distance for sensor = max_safe_dist = 0.14
	Distance from object at which object avoidance = max_avoid_dist = 0.2
	Desired Distance traveled by flock = desired_dist_traveled = 2.2 (Its unit are in meters)
	Average Ir value of flock where robot should stop depends upon number_robots in experiment. Ir value detected by flock is performance
	of flock ad it is interpolated as follows.
	target_flock_perf = interp(max_cluster_size, [1, 15], [0.4, 0.6])
	Desired compactness value depends upon number of robots. 
	Target_spat = interp(max_cluster_size, [1, 15], [0.3, 0.5])
Note: To terminate the script condition can be applied on dist_traveled or execu_time or both of them.
######################################################################################



####################### Swarm Foraging using AIIS Algorithm ######################

1.Make sure to have CoppeliaSim running, with followig scene loaded: 
Scenes/Foraging_behavior_e_puck_Swarm.ttt

2.The client side script "Foraging_Objects_Swarm_Robots.py" depends on following python scripts:
sim.py, simConst.py, robot_epuck.py, utlis.py, robot_generation.py, and the remote API library available
in CoppeliaRobotics\CoppeliaSimPro\programming\remoteApiBindings\lib\lib\Windows.

3. To load the robots model of coppeliasim in python run the "robot_generation.py" and input number of robots that user placed in Coppeliasim scene. 
This program write scene properties in a file with the name of "robot_file", By doing so, there is no need to load scene for each experiment of Flocking.

4. Now run the "Foraging_Objects_Swarm_Robots.py" file and enter the total execution time for experiment.

5.There are some parameters which can be changed according to your requiremwnt in script.
Parameters:
	Defaut Number of food is 20, Enter total number of food total_num_food = 20 (Change number of food in scene to change total_num_food)
	Target Percentage of food  = target_num_food = 80 %
	Execution time of experiment = execu_time = 600 sec
	Maximum Sensor range = sensor_range = 0.25
	Minimum safe distance for sensor = min_safe_dist = 0.1
	Maximum safe distance for sensor = max_safe_dist = 0.15
	Distance from object at which object avoidance = max_avoid_dist = 0.2
	Energy of Robot at start = energy_robot = 500 
	Energy Consumed by robot at each step = energy_unit_consum = 1
	Addition in energy when an object is picked = energy_addition = 200 
 -------- Von-Neumann neighborhood function parameters for robot monokines ------------#
mono_rob_val = 100
mono_rob_diff = 0.35
mono_rob_c1 = 0.15
mono_rob_c2 = 0.025
  -------- Von-Neumann neighborhood function parameters for food chemokines ------------#
chem_food_val = 100
chem_food_diff = 0.35
chem_food_c1 = 0.125
chem_food_c2 = 0.021
Note: To terminate the script condition can be applied on total_food_coll or execu_time or both of them.
######################################################################################



####################### Swarm Object Clustering using AIIS Algorithm ######################

1.Make sure to have CoppeliaSim running, with followig scene loaded: 
Scenes/Object_Clustering_Swarm_Robotics.ttt

2.The client side script "Object_Clustering_Swarm_Robots.py" depends on following python scripts:
sim.py, simConst.py, robot_epuck.py, utlis.py, robot_generation.py, and the remote API library available
in CoppeliaRobotics\CoppeliaSimPro\programming\remoteApiBindings\lib\lib\Windows.

3. To load the robots model of coppeliasim in python run the "robot_generation.py" and input number of robots that user placed in Coppeliasim scene. 
This program write scene properties in a file with the name of "robot_file", By doing so, there is no need to load scene for each experiment of Flocking.

4. Now run the "Foraging_Objects_Swarm_Robots.py" file and enter the total execution time for experiment.

5.There are some parameters which can be changed according to your requiremwnt in script.
Parameters:
	Enter total number of food total_num_food = 40 (Change number of food in scene to change total_num_food)
	Target Percentage of food  = target_num_food = 80 %
	Execution time of experiment = execu_time = 600 sec
	Maximum Sensor range = sensor_range = 0.25
	Minimum safe distance for sensor = min_safe_dist = 0.1
	Maximum safe distance for sensor = max_safe_dist = 0.15
	Distance from object at which object avoidance = max_avoid_dist = 0.2
 -------- Von-Neumann neighborhood function parameters for robot monokines ------------#
mono_rob_val = 100
mono_rob_diff = 0.32
mono_rob_c1 = 0.127
mono_rob_c2 = 0.0205
  -------- Von-Neumann neighborhood function parameters for food chemokines ------------#
chem_food_val = 100
chem_food_diff = 0.35
chem_food_c1 = 0.125
chem_food_c2 = 0.021
Note: To terminate the script condition can be applied on total_food_coll or execu_time or both of them.
######################################################################################
