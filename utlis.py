# uncompyle6 version 3.7.4
# Python bytecode 3.8 (3413)
# Decompiled from: Python 2.7.17 (default, Sep 30 2020, 13:38:04) 
# [GCC 7.5.0]
# Warning: this version of Python has problems handling the Python 3 "byte" type in constants properly.

# Embedded file name: D:\Python_Vrep\utlis.py
# Compiled at: 2021-08-12 12:51:35
# Size of source mod 2**32: 6198 bytes
import numpy as np
import itertools

def center_mass_dist(rob_position):
    rob_position = np.array(rob_position)
    x_sum = np.sum(rob_position[:, 0]) / len(rob_position)
    y_sum = np.sum(rob_position[:, 1]) / len(rob_position)
    cen_mass = np.array([x_sum, y_sum])
    diff = rob_position - cen_mass
    summa = diff * diff
    dist_robots = np.sqrt(summa.sum(axis=1))
    return (dist_robots, cen_mass)


clamp = lambda n, minn, maxn: max(min(maxn, n), minn)

def cluster(robots_pos, robots_det_rob):
    robots_detrob_dic = dict()
    robots_pos_dic = dict()
    for i, robot_detrob in enumerate(robots_det_rob):
        if robot_detrob:
            robots_detrob_dic[i] = len(robot_detrob)
            robots_pos_dic[i] = robots_pos[i]
        print('robots_position dic', robots_pos_dic)
        print('robots_det_rob dic', robots_pos_dic)
        return (robots_pos_dic, robots_detrob_dic)


def spatial_fitness(dist_rob):
    dist_rob = dist_rob / np.max(dist_rob)
    dist_rob = (1 - dist_rob) / len(dist_rob)
    fitness = np.sum(dist_rob)
    return fitness


def fitness_aggregation(robots_position):
    avg_dis, com_point = center_mass_dist(robots_position)
    fitnesn_step = spatial_fitness(avg_dis)
    return (fitnesn_step, com_point)


def angles_calcu(sensor_x, sensor_y, robot_x, robot_y):
    grad_x = sensor_x - robot_x
    grad_y = sensor_y - robot_y
    angles = np.arctan2(grad_y, grad_x)
    mag = np.sqrt(grad_x ** 2 + grad_y ** 2)
    return (mag, angles)


def resultant_vector(magnitude, angles):
    fx = magnitude * np.cos(angles)
    fy = magnitude * np.sin(angles)
    fres_x = np.sum(fx)
    fres_y = np.sum(fy)
    return (fres_x, fres_y)


def gradient_rob_nei(robot_handles, sensor_acq_handle, robot_present_loc, concerned_rob_alocation):
    if not concerned_rob_alocation:
        detected_robot = set(robot_handles).intersection(set(sensor_acq_handle))
        detected_rob_num = [robot_handles.index(m) for m in detected_robot]
        nei_rob_loc = [robot_present_loc[rob_num] for rob_num in detected_rob_num]
        response = True in nei_rob_loc
        leader = detected_rob_num[nei_rob_loc.index(True)] if response else -1
    else:
        response = False
        leader = -1
    return response, leader


def dist_btw_points(point1, point2):
    diff = point1 - point2
    summa = diff * diff
    travel_distance = np.sqrt(summa.sum(axis=0))
    return travel_distance


def fitness(detected, dc_loc):
    diff = detected - dc_loc
    summa = diff * diff
    distance = np.sqrt(summa.sum(axis=1))
    return distance


def cluster_count(robs_pos, sensor_range):
    robs_pos = np.array(robs_pos)
    neigh_rob = list()
    for i in range(len(robs_pos)):
        dist = fitness(robs_pos, robs_pos[i])
        neigh_rob.append(np.where(dist < sensor_range)[0])
    else:
        return neigh_rob


def remove_common(lst):
    return [list(i) for i in {*[tuple(sorted(i)) for i in lst]}]


def find_maxLen_list(lst):
    maxList = max(lst, key=len)
    maxLength = len(maxList)
    return (maxList, maxLength)


def cluster_count(robs_pos, sensor_range):
    robs_pos = np.array(robs_pos)
    neigh_rob = list()
    for i in range(len(robs_pos)):
        dist = fitness(robs_pos, np.array(robs_pos[i]))
        neigh_rob.append(list(np.where(dist < sensor_range)[0]))
    else:
        return neigh_rob

# number of points around a specific point
def robots_range(center_clusters, robs_pos, sensor_range):
    robs_pos = np.array(robs_pos)
    neigh_rob = list()
    for i in range(len(center_clusters)):
        dist = fitness(center_clusters[i], robs_pos)
        neigh_rob.append(list(np.where(dist < sensor_range)[0]))
    else:
        return neigh_rob


def flock_members(rob_p, sens_range):
    neigh_comb = []
    final_clusters = []
    neigh_rob = cluster_count(rob_p, sens_range)
    print('neigh_rob', neigh_rob)
    comb_len = [len(comb) for comb in neigh_rob]
    max_comb = max(comb_len)
    if max_comb > 1:
        for i in range(len(neigh_rob)):
            if len(neigh_rob[i]) != 1:
                neigh_comb.append(set(neigh_rob[i]))
    else:
        neigh_comb = [set(single) for single in neigh_rob]
    for base_cluster in neigh_comb:
        for next_cluster in neigh_comb:
            if len(base_cluster.intersection(next_cluster)) != 0:
                base_cluster = base_cluster.union(next_cluster)
            final_clusters.append(list(base_cluster))
        else:
            final_clusters = remove_common(final_clusters)
            final_clusters_len = [len(sub_clus) for sub_clus in final_clusters]
            largest_cluster, len_largest_cluster = find_maxLen_list(final_clusters)
            print('Largest Cluster is', largest_cluster)
        return (final_clusters, final_clusters_len, largest_cluster, len_largest_cluster)


def cluster_formed(rob_p, sens_range):
    neigh_comb = []  # group of robots in a certain sensor range of robot
    final_clusters = []
    neigh_rob = cluster_count(rob_p, sens_range)
    comb_len = [len(comb)for comb in neigh_rob]
    max_comb = max(comb_len)
    if max_comb > 1:
        for i in range(len(neigh_rob)): # remove single robot from list of combination
            if len(neigh_rob[i]) != 1 :
                neigh_comb.append(set(neigh_rob[i]))
    else:
        neigh_comb = [set(single) for single in neigh_rob]
    for base_cluster in neigh_comb:
    #print('base_cluster', base_cluster)
        for next_cluster in neigh_comb:
            if len(base_cluster.intersection(next_cluster)) != 0:
                base_cluster = base_cluster.union(next_cluster)
                #print('next_cluster', next_cluster)
                #print('base_cluster', base_cluster)
                #neigh_comb.remove(next_cluster)
                #print('neigh_comb', neigh_comb)
        #print(list(base_cluster))
        final_clusters.append(list(base_cluster))
    final_clusters = remove_common(final_clusters)
    final_clusters_len = [len(sub_clus) for sub_clus in final_clusters]
    print(final_clusters)
    print(final_clusters_len)
    largest_cluster, len_largest_cluster = find_maxLen_list(final_clusters)
    print('Largest Cluster is', largest_cluster)
    return (final_clusters, final_clusters_len, largest_cluster, len_largest_cluster)


def cartesian_im_trans(ran_x, ran_y, size_pix, coord_obj_plac):
    ran_x_pix = round(ran_x / size_pix)
    ran_y_pix = round(ran_y / size_pix)
    #chemo_array = np.zeros([ran_x_pix, ran_y_pix])  # mask
    pix_obj_plac = np.array(coord_obj_plac) / size_pix
    pix_obj_plac = pix_obj_plac.astype(int)
    return ran_x_pix, ran_y_pix, pix_obj_plac


def chemotaxis_gradient_forging(chem_im, pos_bacteria, neighbor_hood, c1, c2, diffusion, rate):  # Most suitable c1 = 0.125, c2 = 0.02, diffusion = 0.25
    arena_x, arena_y = chem_im.shape  # extract size of mask
    x_arr = np.arange(2, arena_x - 2)
    y_arr = np.arange(2, arena_y - 2)
    all_idx = list(itertools.product(x_arr, y_arr))
    C = np.array(chem_im)
    for i, pos_bac in enumerate(pos_bacteria):
        chem_im[pos_bac[0]][pos_bac[1]] = chem_im[pos_bac[0]][pos_bac[1]] + rate
    for idx in all_idx:
        k = idx[0]
        j = idx[1]
        if neighbor_hood == 'von_neumann':
            chem_im[k][j] = 0.5 * chem_im[k][j] + c1 * (
                    C[k + 1][j] + C[k - 1][j] + C[k][j + 1] + C[k][j - 1]) + c2 * (
                                            C[k - 2][j] + C[k - 1][j - 1] + C[k][j - 2] + C[k + 1][j - 1] +
                                            C[k + 2][j] + C[k + 1][j + 1] + C[k][j + 2] + C[k - 1][
                                                 j + 1]) - diffusion * chem_im[k][j]
        elif neighbor_hood == 'moore':
            chem_im[k][j] = 0.5 * chem_im[k][j] + c1 * (
                    C[k + 1][j] + C[k - 1][j] + C[k][j + 1] + C[k][j - 1]) + c2 * (
                                            C[k - 1][j - 1] + C[k + 1][j - 1] +
                                            C[k + 1][j + 1] + C[k - 1][j + 1]) - diffusion * chem_im[k][j]
    return chem_im


# Robot finding food or things in arena by sensing chemicals secreted by food.
def chemotaxis_gradient_foraging(chemokines, monokines, rob_pos_mask, grad_dir,  mono_factor):
    x = rob_pos_mask[0]
    y = rob_pos_mask[1]
    gradx = grad_dir * (chemokines[x + 1][y] - chemokines[x - 1][y]) - mono_factor * (monokines[x + 1][y] - monokines[x - 1][y])
    grady = grad_dir * (chemokines[x][y + 1] - chemokines[x][y - 1]) - mono_factor * (monokines[x][y + 1] - monokines[x][y - 1])
    gradxy = grad_dir * (chemokines[x + 1][y + 1] - chemokines[x - 1][y - 1]) - mono_factor * (monokines[x + 1][y + 1] - monokines[x - 1][y - 1])
    gradyx = grad_dir * (chemokines[x - 1][y + 1] - chemokines[x + 1][y - 1]) - mono_factor * (monokines[x - 1][y + 1] - monokines[x + 1][y - 1])
    all_grad = np.array([gradx, grady, gradxy, gradyx])
    max_grad = np.max(abs(all_grad))
    if max_grad == 0.0:  # To overcome infinite array
        all_grad = np.array([0.0] * 4)
    else:  # maximum gradient is calculated
        all_grad = all_grad / (max_grad * 8)
    return all_grad, max_grad


# Average filter to apply on chemotaxis on robot positions
def average_filter(chemo_array, rob_pos_im):
    k = rob_pos_im[0]
    j = rob_pos_im[1]
    avg_chemo = 1/9 * (chemo_array[k][j] + chemo_array[k + 1][j] + chemo_array[k - 1][j] + chemo_array[k][j + 1] + chemo_array[k][j - 1] +
            chemo_array[k - 2][j] + chemo_array[k - 1][j - 1] + chemo_array[k][j - 2] + chemo_array[k + 1][j - 1] +
            chemo_array[k + 2][j] + chemo_array[k + 1][j + 1] + chemo_array[k][j + 2] + chemo_array[k - 1][j + 1])
    return avg_chemo


# Evaluation of Foraging
def coll_items(n_items, total_food_coll, coll_items_time, time_passed):
    n_items_parts = [int(n_items * i/10) for i in range(11)]
    #print('n_items_parts')
    perc_time = [time_passed[coll_items_time.index(x)] for x in n_items_parts if x < total_food_coll]
    return perc_time