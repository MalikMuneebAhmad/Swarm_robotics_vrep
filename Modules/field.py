import numpy as np
import itertools
import math
import matplotlib.pyplot as plt
import random


class placement:

    def __init__(self, arena_x, arena_y, num_cell):
        self.arena_x = arena_x  # x-axis of arena
        self.arena_y = arena_y  # y-axis of arena
        self.cell_x = list()
        self.cell_y = list()
        self.pad_x = round(self.arena_x * (0.04))  # Padding of 4%
        self.pad_y = round(self.arena_y * (0.04))
        self.num_cell = num_cell

    def uniform(self, density_dc):
        lenx = (self.arena_x - self.pad_x - 1) - (self.pad_x)
        leny = (self.arena_y - self.pad_y - 1) - (self.pad_y)
        n_dc = int(density_dc * (self.arena_x * self.arena_y))
        #print('No of DC', n_dc)
        if lenx >= leny:
            w = lenx
            h = leny
        else:
            w = leny
            h = lenx
        nx = math.sqrt((w * n_dc / h) + ((w - h) / (2 * h)) ** 2)
        ny = n_dc / nx
        x_arr = np.linspace(self.pad_x, self.arena_x - self.pad_x - 1, round(nx), dtype=int)
        y_arr = np.linspace(self.pad_x, self.arena_y - self.pad_y - 1, round(ny), dtype=int)
        pos = list(itertools.product(x_arr, y_arr))  # Position in the form of oerder pair
        self.cell_x = [e[0] for e in pos]
        self.cell_y = [e[1] for e in pos]
        self.num_cell = len(x_arr) * len(y_arr)
        return self.num_cell, self.cell_x, self.cell_y

    def cluster(self, num_cell, exp_span, desire_loc = None): # exp_span means region in which cells will be placed
        if desire_loc == None: # if no desire mean position will be provided then random pos will be selected
            x = random.randrange(self.pad_x, (self.arena_x - self.pad_x - 1))
            y = random.randrange(self.pad_y, (self.arena_y - self.pad_y - 1))
        else:
            x = desire_loc[0]
            y = desire_loc[1]
        possible_cellx = np.array(list(range(x - exp_span, x + exp_span +1)))
        possible_celly = np.array(list(range(y - exp_span, y + exp_span+1)))
        possible_cell_loc = list(itertools.product(possible_cellx, possible_celly))
        cell_pos = random.sample(possible_cell_loc, num_cell)  # Select Randomly Location of Defined Number of Cells
        self.cell_x = [cell[0] for cell in cell_pos]  # Cell X-Coordinates
        self.cell_y = [cell[1] for cell in cell_pos]  # Cell Y-Coordinates
        return self.cell_x, self.cell_y

    def random(self, num_cell):
        self.cell_x = random.sample(range(self.pad_x, self.arena_x - self.pad_x - 1), num_cell)
        self.cell_y = random.sample(range(self.pad_y, self.arena_y - self.pad_y - 1), num_cell)
        return self.cell_x, self.cell_y


'''dc = placement(80,70)
dc_x, dc_y = dc.cluster(1, 3)
plt.plot(dc_x, dc_y, 'b.')
plt.show()'''
