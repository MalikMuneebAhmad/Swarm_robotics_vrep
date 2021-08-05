import numpy as np


def center_mass_dist(rob_position):  # function for calculating distances of mass from center of mass
    rob_position = np.array(rob_position)
    x_sum = np.sum(rob_position[:, 0]) / len(rob_position)
    y_sum = np.sum(rob_position[:, 1]) / len(rob_position)
    cen_mass = np.array([x_sum, y_sum])
    diff = rob_position - cen_mass
    summa = (diff * diff)
    dist_robots = np.sqrt(summa.sum(axis=1))
    return dist_robots


clamp = lambda n, minn, maxn: max(min(maxn, n), minn)


def cluster(robots_pos, robots_det_rob):  # Calculate position and detected robot of a robot
    robots_detrob_dic = dict()
    robots_pos_dic = dict()
    for i, robot_detrob in enumerate(robots_det_rob):
        if robot_detrob:
            robots_detrob_dic[i] = len(robot_detrob)
            robots_pos_dic[i] = robots_pos[i]
    print('robots_position dic', robots_pos_dic)
    print('robots_det_rob dic', robots_pos_dic)
    return robots_pos_dic, robots_detrob_dic


# spatial inter-robot relationships
def spatial_fitness(dist_rob):  # distance of robots from center of mass
    dist_rob = dist_rob/np.max(dist_rob)
    dist_rob = (1 - dist_rob)/len(dist_rob)
    fitness = np.sum(dist_rob)
    return fitness


# Fitness calculated from center of mass for one simulation step
def fitness_aggregation(robots_position):
    avg_dis = center_mass_dist(robots_position)  # outpugt Normalize avg distances from Cnter of mass
    fitnesn_step = spatial_fitness(avg_dis)
    return fitnesn_step

def angles_calcu(sensor_x, sensor_y, robot_x, robot_y): # function for calculating gradient or angles
    grad_x = (sensor_x - robot_x)
    grad_y = (sensor_y - robot_y)
    angles = np.arctan2(grad_y, grad_x)
    #print('Calculation of angles in functon', angles)
    mag = np.sqrt(grad_x**2 + grad_y**2)
    return mag, angles

def resultant_vector(magnitude, angles): #  vectors summation
    fx = magnitude * np.cos(angles)
    fy = magnitude * np.sin(angles)
    #print('force x compo = ', fx)
    #print('force y compo = ', fy)
    fres_x = np.sum(fx)
    fres_y = np.sum(fy)
    return fres_x, fres_y


# inputs are: all robot handles, handle acquire by concern robot, State of all robots present in gradient or black
# Output: Binary - true when any detecting robot in gradient area, false when all detecting robots in black area
# Function will be utilized for flocking behavior
def gradient_rob_nei(robot_handles, sensor_acq_handle, robot_present_loc, concerned_rob_alocation):
    if not concerned_rob_alocation:  # robot in black area
        detected_robot = set(robot_handles).intersection(set(sensor_acq_handle))  # detected thing is robot
        detected_rob_num = [robot_handles.index(m) for m in detected_robot]  # extract robot number from handle
        nei_rob_loc = [robot_present_loc[rob_num] for rob_num in detected_rob_num]
        response = True in nei_rob_loc
        leader = detected_rob_num[nei_rob_loc.index(True)] if response else -1
    else:
        response = False
        leader = -1
    return response, leader