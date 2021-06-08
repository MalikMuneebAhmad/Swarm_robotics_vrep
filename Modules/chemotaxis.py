import numpy as np
import cv2
import itertools
import matplotlib.pyplot as plt


class Chemotaxis:
    rate = 100

    def __init__(self, arena_x, arena_y, num_bac) -> object:
        self.arena_x = arena_x
        self.arena_y = arena_y
        self.num_bac = num_bac  # Number of Bacteria
        self.chemokines = np.zeros((self.arena_x, self.arena_y))
        self.all_coordinates = list(itertools.product(np.arange(self.arena_x), np.arange(self.arena_y)))
        self.internal_damage = np.zeros((self.arena_x, self.arena_y))

    def chemo_attractants_old(self, bacteria_alive, bac_x,
                          bac_y):  # Bacteria Release chemical Chemokine
        # which attract agents and  Immune Cell, Immune cell and T-cell Releases Monokine or Cytokine to attract
        # other immune cell
        for i in range(self.num_bac):
            if bacteria_alive[i]:
                x = bac_x[i]
                y = bac_y[i]
                self.chemokines[x][y] = self.chemokines[x][y] + Chemotaxis.rate
        # Placing r = 2 neighborhood value around An Immune Cell or Bacterium in Monokine or Chemokine Frame
        C = self.chemokines
        r = 2
        for k in range(r, self.arena_x - 3):
            for j in range(r, self.arena_y - 3):
                self.chemokines[k][j] = 0.5 * self.chemokines[k][j] + 0.125 * (
                        C[k + 1][j] + C[k - 1][j] + C[k][j + 1] + C[k][j - 1]) + 0.02 * (
                                                C[k + 2][j] + C[k - 2][j] + C[k][j + 2] + C[k][j - 2]) - 0.1 * \
                                        self.chemokines[k][j];
        return self.chemokines

    def chemo_attractants(self,neighbor_hood, bac_x, bac_y, c1, c2, diffusion): # Most suitable c1 = 0.125, c2 = 0.02, diffusion = 0.25
        x_arr = np.arange(2, self.arena_x-2)
        y_arr = np.arange(2, self.arena_y-2)
        all_idx = list(itertools.product(x_arr, y_arr))
        self.chemokines[[bac_x], [bac_y]] = self.chemokines[[bac_x], [bac_y]] + Chemotaxis.rate
        C = np.array(self.chemokines)
        for idx in all_idx:
            k = idx[0]
            j = idx[1]
            if neighbor_hood == 'von_neumann':
                self.chemokines[k][j] = 0.5 * self.chemokines[k][j] + c1 * (
                            C[k + 1][j] + C[k - 1][j] + C[k][j + 1] + C[k][j - 1]) + c2 * (
                                                    C[k - 2][j] + C[k - 1][j - 1] + C[k][j - 2] + C[k + 1][j - 1] +
                                                    C[k + 2][j] + C[k + 1][j + 1] + C[k][j + 2] + C[k - 1][j + 1]) - diffusion * self.chemokines[k][j]
            elif neighbor_hood == 'moore':
                self.chemokines[k][j] = 0.5 * self.chemokines[k][j] + c1 * (
                            C[k + 1][j] + C[k - 1][j] + C[k][j + 1] + C[k][j - 1]) + c2 * (
                                                    C[k - 1][j - 1] + C[k + 1][j - 1] +
                                                    C[k + 1][j + 1] + C[k - 1][j + 1]) - diffusion * self.chemokines[k][j]
        return self.chemokines

    '''def chemo_attractants(self, bacteria_alive, bac_x, bac_y):
        # Bacteria Release chemical Chemokine which attract agents and  Immune Cell, Immune cell and
        # T-cell Releases Monokine or Cytokine to attract other immune cell
        x_arr = np.arange(2, self.arena_x-2)
        y_arr = np.arange(2, self.arena_y-2)
        all_idx = list(itertools.product(x_arr, y_arr))
        print('all_idx', all_idx)
        #C = np.copy(self.chemokines)
        self.chemokines[[bac_x],[bac_y]] = self.chemokines[[bac_x],[bac_y]] + Chemotaxis.rate
        #print(np.amax(C))
        C = np.array(self.chemokines)
        for idx in all_idx:
            print('idx', idx)

            #k = idx[0]
            #j = idx[1]
            #self.chemokines[k][j] = (0.5 * self.chemokines[k][j]) + 0.125 * (
                    #C[k + 1][j] + C[k - 1][j] + C[k][j + 1] + C[k][j - 1]) + 0.02 * (
                                            #C[k + 1][j] + C[k - 1][j] + C[k][j + 1] + C[k][j - 1])
                                    #- (0.1 * self.chemokines[k][j])
            #self.chemokines[k][j] = 0.5 * C[k][j] + 0.25 * (C[k + 1][j] + C[k - 1][j] + C[k][j + 1] + C[k][j - 1]) + 0.125 * (C[k - 2][j] + C[k - 1][j - 1] + C[k][j-2] + C[k + 1][j-1] + C[k + 2][j] + C[k + 1][j + 1] + C[k][j + 2] + C[k - 1][j + 1])

            #my_filter = 1 / 100 * np.array([[2, 12.5, 2], [12.5, 50, 12.5],[2, 12.5, 2]])
            #self.chemokines = cv2.filter2D(self.chemokines, -1, my_filter)
            return C'''

    '''def internal_cellular_effect(self, bacteria_x, bacteria_y, region, intensity):  # Effect of bacteria on host Cells.  Two input parameters are required first is area and second is intensity.
        nei_loc = list()
        for i in range(self.num_bac):
            nei_x = range(max(0, bacteria_x[i] - region), min(self.arena_x, bacteria_x[i] + region + 1))
            nei_y = range(max(0, bacteria_y[i] - region), min(self.arena_y, bacteria_y[i] + region + 1))
            nei_loc += list(itertools.product(nei_x, nei_y))
        for i, single_nei in enumerate(nei_loc):
            # print('{} at location {}'.format(i, single_nei))
            self.internal_damage[single_nei[0]][single_nei[1]] += min(region - abs(single_nei[0] - bacteria_x[0]),
                                                                      region - abs(
                                                                          single_nei[1] - bacteria_y[0])) * (intensity/region)
        remain_pixels = set(self.all_coordinates) - set(nei_loc)
        for pixel in remain_pixels:
            if self.internal_damage[pixel[0]][pixel[1]] == 0:
                continue
            else:
                self.internal_damage[pixel[0]][pixel[1]] = max(0, self.internal_damage[pixel[0]][pixel[1]]-0.1)
        return self.internal_damage

# Code to test Chemotaxis behavior
bacx = [5, 5, 5]
bacy = [12, 24, 36]
is_alive = [1, 1, 1]
arena = int(50)
num = 3
a = Chemotaxis(arena, num)
for j in range(20):
    chem = a.chemo_attractants(is_alive, bacx, bacy)
    plt.title('Chemotaxis Plot')
    plt.xlabel('Y')
    plt.ylabel('X')
    plt.imshow(chem, origin='lower')
plt.show()'''
