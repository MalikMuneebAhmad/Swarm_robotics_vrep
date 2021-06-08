import numpy as np
import random
import matplotlib.pyplot as plt
import itertools
from matplotlib import colors


class Bacteria():
    p = 0.125

    def __init__(self, arena_x, arena_y, num_bac):
        self.bac_pattern = str()
        self.arena_x = arena_x
        self.arena_y = arena_y
        self.bacteria_x = list()
        self.bacteria_y = list()
        self.pad_x = round(self.arena_x * (0.04))  # Padding of 4%
        self.pad_y = round(self.arena_y * (0.04))
        self.bacteria_location = list()
        self.num_bac = num_bac
        self.is_alive = [1] * num_bac
        self.all_coordinates = list(itertools.product(np.arange(self.arena_x), np.arange(self.arena_y)))
        self.internal_damage = np.zeros([arena_x, arena_y])
        self.normalize_internal_damage = np.zeros([arena_x, arena_y])

    def cluster(self, exp_span, desire_loc=None):  # exp_span means region in which cells will be placed
        if desire_loc == None:  # if no desire mean position will be provided then random pos will be selected
            x = random.randrange(self.pad_x, (self.arena_x - self.pad_x - 1))
            y = random.randrange(self.pad_y, (self.arena_y - self.pad_y - 1))
        else:
            x = desire_loc[0]
            y = desire_loc[1]
        possible_cellx = np.array(list(range(x - exp_span, x + exp_span + 1)))
        possible_celly = np.array(list(range(y - exp_span, y + exp_span + 1)))
        possible_cell_loc = list(itertools.product(possible_cellx, possible_celly))
        cell_pos = random.sample(possible_cell_loc, self.num_bac)  # Select Randomly Location of Defined Number of Cells
        self.bacteria_x = [cell[0] for cell in cell_pos]  # Cell X-Coordinates
        self.bacteria_y = [cell[1] for cell in cell_pos]  # Cell Y-Coordinates
        self.bacteria_location = [[self.bacteria_x[i], self.bacteria_y[i]] for i in range(len(self.bacteria_x))]
        return self.bacteria_x, self.bacteria_y, self.bacteria_location

    def random(self):
        self.bacteria_x = random.sample(range(self.pad_x, self.arena_x - self.pad_x - 1), self.num_bac)
        self.bacteria_y = random.sample(range(self.pad_y, self.arena_y - self.pad_y - 1), self.num_bac)
        return self.bacteria_x, self.bacteria_y

    def move(self, mode):  # Move a Population of Bacteria Randomly in arena
        movement = {'Mild': 1, 'Active': 2, 'Hyperactive': 3}  # Possible activity of bacteria
        step_size = movement[mode]
        for i in range(len(self.bacteria_x)):
            r = random.random()
            if self.is_alive[i]:
                if r < Bacteria.p:  # Bacteria move Upper-Left
                    self.bacteria_x[i] -= step_size
                    self.bacteria_y[i] += step_size
                    # print('Upper-Left')
                    if self.bacteria_x[i] <= 1:
                        self.bacteria_x[i] += 5
                        # print('Bounce Back')
                    elif self.bacteria_y[i] >= self.arena_y - 2:
                        self.bacteria_y[i] -= 5
                        # print('Bounce Back')
                elif Bacteria.p <= r < 2 * Bacteria.p:  # Bacteria move UP
                    self.bacteria_y[i] += step_size
                    # print('Up')
                    if self.bacteria_y[i] >= self.arena_y - 2:
                        self.bacteria_y[i] -= 5
                        # print('Bounce Back')
                elif 2 * Bacteria.p <= r < 3 * Bacteria.p:  # Bacteria move Upper-Right
                    self.bacteria_x[i] += step_size
                    self.bacteria_y[i] += step_size
                    # print('Upper-Right')
                    if self.bacteria_x[i] >= self.arena_x - 2:
                        self.bacteria_x[i] -= 5
                        # print('Bounce Back')
                    elif self.bacteria_y[i] >= self.arena_y - 2:
                        self.bacteria_y[i] -= 5
                        # print('Bounce Back')
                elif 3 * Bacteria.p <= r < 4 * Bacteria.p:  # Bacteria move Right
                    self.bacteria_x[i] += step_size
                    # print('Right')
                    if self.bacteria_x[i] >= self.arena_x - 2:
                        self.bacteria_x[i] -= 5
                        # print('Bounce Back')
                elif 4 * Bacteria.p <= r < 5 * Bacteria.p:  # Bacteria move Down-Rght
                    self.bacteria_x[i] += step_size
                    self.bacteria_y[i] -= step_size
                    # print('Down-Rght')
                    if self.bacteria_x[i] >= self.arena_x - 2:
                        self.bacteria_x[i] -= 5
                        # print('Bounce Back')
                    elif self.bacteria_y[i] <= 1:
                        self.bacteria_y[i] += 5
                        # print('Bounce Back')
                elif 5 * Bacteria.p <= r < 6 * Bacteria.p:  # Bacteria move Down
                    self.bacteria_y[i] -= step_size
                    # print('Down')
                    if self.bacteria_y[i] <= 1:
                        self.bacteria_y[i] += 5
                        # print('Bounce Back')
                elif 6 * Bacteria.p <= r < 7 * Bacteria.p:  # Bacteria move Left-Down
                    self.bacteria_x[i] -= step_size
                    self.bacteria_y[i] -= step_size
                    # print('Down-Left')
                    if self.bacteria_x[i] <= 1:
                        self.bacteria_x[i] += 5
                        # print('Bounce Back')
                    elif self.bacteria_y[i] <= 1:
                        self.bacteria_y[i] += 5
                        # print('Bounce Back')
                elif 7 * Bacteria.p <= r < 8 * Bacteria.p:  # Bacteria move Left
                    self.bacteria_x[i] -= step_size
                    # print('Left')
                    if self.bacteria_x[i] <= 1:
                        self.bacteria_x[i] += 5
                        # print('Bounce Back')
        self.bacteria_location = [[self.bacteria_x[i], self.bacteria_y[i]] for i in range(len(self.bacteria_x))]
        return self.bacteria_x, self.bacteria_y, self.bacteria_location

    def internal_cellular_effect(self, region, intensity):  # Effect of bacteria on host Cells.  Two input parameters are required first is area and second is intensity.
        nei_loc = list()
        for i in range(self.num_bac):
            #print('Num_bac_Error', self.num_bac)
            nei_x = range(max(0, self.bacteria_x[i] - region), min(self.arena_x, self.bacteria_x[i] + region + 1))
            nei_y = range(max(0, self.bacteria_y[i] - region), min(self.arena_y, self.bacteria_y[i] + region + 1))
            current_bacteria_nei = list(itertools.product(nei_x, nei_y))
            nei_loc += current_bacteria_nei
            for _, single_nei in enumerate(current_bacteria_nei):
                # print('{} at location {}'.format(i, single_nei))
                self.internal_damage[single_nei[0]][single_nei[1]] += min(region - abs(single_nei[0] - self.bacteria_x[i]),
                                                                        region - abs(
                                                                            single_nei[1] - self.bacteria_y[i])) * (intensity/region)
        remain_pixels = set(self.all_coordinates) - set(nei_loc)
        for pixel in remain_pixels:
            if self.internal_damage[pixel[0]][pixel[1]] == 0:
                continue
            else:
                self.internal_damage[pixel[0]][pixel[1]] = max(0, self.internal_damage[pixel[0]][pixel[1]]-0.1 * self.internal_damage[pixel[0]][pixel[1]])
        #self.normalize_internal_damage = self.internal_damage/(np.amax(self.internal_damage) * 10)
        return self.internal_damage

    def arrayupdation(self, marker):  # Bacteria Array Updation
        plt.plot(self.bacteria_y, self.bacteria_x, marker)


# Code for testing the Bacteria Cell class
'''a = Bacteria(80, 70, 10)
bac_x, bac_y = a.random()
plt.title('Bacteria placement')
plt.xlabel('Bacteria-X')
plt.ylabel('Bacteria-Y')
a.arrayupdation('b.')
plt.show()'''

'''for x in range(10):
    if x == 0:
        a = Bacteria(100, 80, 10)
        bac_x, bac_y = a.cluster(4, [70, 60])
    else:
        if x == 6:
            a.is_alive = [1, 1, 0, 0, 0, 1, 1, 1, 0, 0]
        a.move('active')
        plt.title('Bacteria Random Movement in Tissue ' + str(x))
        plt.xlabel('Bacteria-X')
        plt.ylabel('Bacteria-Y')
        a.arrayupdation('r*')
        plt.show()
        print(x)'''
