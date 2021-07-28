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