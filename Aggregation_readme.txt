# Make sure to have CoppeliaSim running, with followig scene loaded:
#
# scenes/Aggregation_e_puck_Swarm.ttt
#
# The client side "Aggregation_Swarm_Robots_Avoiding_Objects.py" depends on:
#
# sim.py, simConst.py, robot_epuck.py, utlis.py, robot_generation.py, regulation_mechanism.py, and the remote API library available
# in programming/remoteApiBindings/lib/lib

2. To load the robots model of coppeliasim in python run the "robot_generation.py" and input number of robots that user placed in scene. 
This program wrtr scene properties in a file with the name of "robot_file", By doing so, there is no need to load scene for each experiment of aggregation.

3. Now run the "Aggregation_Swarm_Robots_Avoiding_Objects.py" file. First of all file load robots models from file "robot_file".

4. Enter the total time for ehich you want to execute the script and enter diffusion rate.

There are some parameters which can be changed according to your requiremwnt.
Parameters:
	Execution time of experiment = execu_time = 600
	Maximum Sensor range = sensor_range = 0.25
	Minimum safe distance for sensor = min_safe_dist = 0.1
	Maximum safe distance for sensor = max_safe_dist = 0.2
	Distance from object at which object avoidance = max_avoid_dist = 0.2
	minimum number of required neighboring robots = min_req_nei = 2
	Diffusion rate of danger in Dc regulation = diff_rate = 0.08 
	Each robot must have x neighboring robots = total_inflam = number of robots * x (recommended x = 3)


# Make sure to have CoppeliaSim running, with followig scene loaded:
#
# scenes/pControllerViaRemoteApi.ttt
#
# Do not launch simulation, but run this script
#
# The client side (i.e. this script) depends on:
#
# sim.py, simConst.py, and the remote API library available
# in programming/remoteApiBindings/lib/lib