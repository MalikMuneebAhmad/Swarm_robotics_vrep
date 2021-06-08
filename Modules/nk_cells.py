import numpy as np
import random
import matplotlib.pyplot as plt
import itertools


class Immune_cells:
    p = 0.125

    def __init__(self, arena_x, arena_y, num_cells, type_im_cell):
        if type_im_cell == 'T-Cell':
            self.tcr = str()
            self.age = 0
        self.tcell_priming_status = False
        self.arena_x = arena_x
        self.arena_y = arena_y
        self.pad_x = round(self.arena_x * (0.04))  # Padding of 4%
        self.pad_y = round(self.arena_y * (0.04))
        self.num_cells = num_cells  # No of Immune Cells
        self.monocytes_x = []  # Monocytes X location (Require)
        self.monocytes_y = []  # Monocytes y location  (Require)
        self.monocytes_loc = []  # (Require)
        self.monocytes_gradientx = np.array([0] * num_cells) # (Require)
        self.monocytes_gradienty = np.array([0] * num_cells)  # (Require)
        self.monocytes_gradientxy = np.array([0] * num_cells)  # (Require)
        self.monocytes_gradientyx = np.array([0] * num_cells)  # (Require)
        self.monokines = np.zeros((self.arena_x, self.arena_y))
        self.cells_presence = [1] * num_cells
        self.bac_pattern = str()  # T-cell suitable for that pattern of bacteria

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
        cell_pos = random.sample(possible_cell_loc, self.num_cells)  # Select Randomly Location of Defined Number of Cells
        self.monocytes_x = [cell[0] for cell in cell_pos]  # Cell X-Coordinates
        self.monocytes_y = [cell[1] for cell in cell_pos]  # Cell Y-Coordinates
        return self.monocytes_x, self.monocytes_y

    def random(self):
        self.monocytes_x = random.sample(range(self.pad_x, self.arena_x - self.pad_x - 1), self.num_cells)
        self.monocytes_y = random.sample(range(self.pad_y, self.arena_y - self.pad_y - 1), self.num_cells)
        return self.monocytes_x, self.monocytes_y

    def gradient(self, chemokines, monokines): #[x1, x2, x3, x4.....xn]/max(x)
        grad = np.zeros([4,self.num_cells]) # Change
        for i in range(self.num_cells):
            x = self.monocytes_x[i]
            y = self.monocytes_y[i]
            grad[0][i] = (chemokines[x+1][y] - chemokines[x-1][y]) - 0.005 * (monokines[x+1][y] - monokines[x-1][y])
            grad[1][i] = (chemokines[x][y+1] - chemokines[x][y-1]) - 0.005 * (monokines[x][y+1] - monokines[x][y-1])
            grad[2][i] = (chemokines[x+1][y+1] - chemokines[x-1][y-1]) - 0.005 * (monokines[x+1][y+1] - monokines[x-1][y-1])
            grad[3][i] = (chemokines[x-1][y+1] - chemokines[x+1][y-1]) - 0.005 * (monokines[x-1][y+1] - monokines[x+1][y-1])
        #print('Gradient', grad)
        mxgrad = np.amax(np.absolute(grad), axis=1)
        #print('Gradient', grad)
        for idx, mx in enumerate(mxgrad):
            if mx == 0:
                grad[idx] = [0] * self.num_cells
            else:
                grad[idx] = grad[idx]/(mxgrad[idx]*8)
        return np.transpose(grad)

    def gradientall(self, chemokines, monokines):  #[x, y, xy, yx]/max
        grad = np.zeros([self.num_cells, 4]) # Change
        for i in range(self.num_cells):
            x = self.monocytes_x[i]
            y = self.monocytes_y[i]
            self.monocytes_gradientx[i] = chemokines[x+1][y] - chemokines[x-1][y] - 0.5 * monokines[x+1][y] - monokines[x-1][y]
            self.monocytes_gradienty[i] = chemokines[x][y+1] - chemokines[x][y-1] - 0.5 * monokines[x][y+1] - monokines[x][y-1]
            self.monocytes_gradientxy[i] = chemokines[x+1][y+1] - chemokines[x-1][y-1] - 0.5 * monokines[x+1][y+1] - monokines[x-1][y-1]
            self.monocytes_gradientyx[i] = chemokines[x-1][y+1] - chemokines[x+1][y-1] - 0.5 * monokines[x-1][y+1] - monokines[x+1][y-1]
            grad[i] = [self.monocytes_gradientx[i], self.monocytes_gradienty[i], self.monocytes_gradientxy[i], self.monocytes_gradientyx[i]]
            #print('Gradient', grad)
        mxgrad = np.amax(np.absolute(grad), axis=1)
        #mxgrad = float(max(np.absolute(grad)))
        for idx, mx in enumerate(mxgrad):
            if mx == 0:
                #print('zero')
                grad[idx] = [0] * 4
            else:
                #print('not zero', mx)
                grad[idx] = grad[idx]/(mx * 8)
        return grad

    def random_movement(self, grad):  # More exploraion
        p = 1/8  # P = [pE, pEN, pN, pNW, pW, pWS, pS, pSE]
        P = [p+grad[0], p+grad[2], p+grad[1], p+grad[3], p-grad[0], p-grad[2], p-grad[1], p-grad[3]]  # Change probability w.r.t Gradient
        simulation = [random.randrange(8) * 1 for _ in range(16)]  # Randomly choose between 0-7 for 16 times
        freq = [simulation.count(i) for i in range(8)]  # Count the occuring of each number(event)
        directions = np.array([float(i*j) for i,j in zip(P, freq)])  # Possibility of each direction
        suitable_dir = np.argmax(directions)  # Select the maximum likelihood direction
        disp = {0: (1, 0), 1: (1, 1), 2: (0, 1), 3: (-1, 1), 4: (-1, 0), 5: (-1, -1), 6: (0, -1), 7: (1, -1)}
        (dx, dy) = disp[suitable_dir]
        #print(self.monocytes_x[0])
        #print(self.monocytes_y[0])
        self.monocytes_x[0] += dx
        self.monocytes_y[0] += dy
        return self.monocytes_x, self.monocytes_y

    def random_movementall(self, gradient):  # More exploraion
        p = 1/8  # P = [pE, pEN, pN, pNW, pW, pWS, pS, pSE]
        for i, grad in enumerate(gradient):
            P = [p+grad[0], p+grad[2], p+grad[1], p+grad[3], p-grad[0], p-grad[2], p-grad[1], p-grad[3]]  # Change probability w.r.t Gradient
            #print('Probability of Immune Cell {0} is {1}'.format(i, P))
            simulation = [random.randrange(8) * 1 for _ in range(16)]  # Randomly choose between 0-7 for 16 times
            freq = [simulation.count(i) for i in range(8)]  # Count the occuring of each number(event)
            directions = np.array([float(i*j) for i, j in zip(P, freq)])  # Possibility of each direction
            suitable_dir = np.argmax(directions)  # Select the maximum likelihood direction
            disp = {0: (1, 0), 1: (1, 1), 2: (0, 1), 3: (-1, 1), 4: (-1, 0), 5: (-1, -1), 6: (0, -1), 7: (1, -1)}
            (dx, dy) = disp[suitable_dir]
            #print(self.monocytes_x[0])
            #print(self.monocytes_y[0])
            self.monocytes_x[i] += dx
            self.monocytes_y[i] += dy
            if self.monocytes_x[i] < 2:  # Condition to overcome out of bound Error
                self.monocytes_x[i] += 3
            elif self.monocytes_x[i] > self.arena_x-2:
                self.monocytes_x[i] -= 3
            if self.monocytes_y[i] < 2:
                self.monocytes_y[i] += 3
            elif self.monocytes_y[i] > self.arena_y-2:
                self.monocytes_y[i] -= 3
            self.monocytes_loc.append([self.monocytes_x[i], self.monocytes_y[i]])
        return self.monocytes_x, self.monocytes_y

    def kill_bac(self, num_bac, bac_x, bac_y, is_alive):
        im_cell_loc = [[self.monocytes_x[i], self.monocytes_y[i]] for i in range(len(self.monocytes_x))]
        #print(im_cell_loc)
        bac_loc = [[bac_x[i], bac_y[i]] for i in range(len(bac_x))]
        for cell in im_cell_loc:
            if cell in bac_loc:
                idx = bac_loc.index(cell)
                #print('Bacteria', bac_loc[idx])
                num_bac -= 1
                bac_x.pop(idx)
                bac_y.pop(idx)
                bac_loc.pop(idx)
                is_alive.pop(idx)
            else:
                continue
        return num_bac, bac_x, bac_y, is_alive

    def arrayupdation(self, marker):  # Macrophages Array Updation
        plt.plot(self.monocytes_y, self.monocytes_x, marker)

    def priming_tcell(self, dc_loc, dc_save_pattern, pool_tcr):
        self.age += 1
        for t_cell in self.monocytes_loc:
            if t_cell in dc_loc and not self.tcell_priming_status: # To prime only unprimed T-cells
                dc_num = dc_loc.index(t_cell)
                tcr_affinity, bac_pattern = Immune_cells.affinity(self.tcr, dc_save_pattern[dc_num])
                if self.age >= 50: # Selection of new TCR
                    self.age = 0
                    self.tcr = random.choice(pool_tcr)
                    pool_tcr.remove(self.tcr)
                    print('TCR HAS BEEN MODIFIED')
                if tcr_affinity >= 3:
                    self.tcell_priming_status = True
                    #print('T-cell is -----------------------Primed')
                    self.bac_pattern = bac_pattern
                    return self.bac_pattern

    def proliferation(self, current_inflammation, change_inflammation):
        if self.tcell_priming_status:
            if current_inflammation > 0.5 and change_inflammation < 0.1:
                self.num_cells += 1
                self.monocytes_x.append(random.choice(self.monocytes_x))
                self.monocytes_y.append(random.choice(self.monocytes_y))
            elif current_inflammation <= 0.5  and self.num_cells > 3:
                rand_tcell = random.randint(0, self.num_cells - 1)
                print('self.num_cells' ,self.num_cells)
                self.monocytes_x.remove(self.monocytes_x[rand_tcell])
                self.monocytes_y.remove(self.monocytes_y[rand_tcell])
                self.num_cells -= 1
                print('T_cells reduced at inflammation', current_inflammation)


    @staticmethod
    def affinity(tcr, dc_pattern):
        best_affinity = 0
        bac_match = str()
        for _ in range(len(dc_pattern)):
            str2 = dc_pattern.pop()
            c = 0
            for i in range(len(tcr)):
                if tcr[i] == str2[i]:
                    c += 1
            if c > best_affinity:
                best_affinity = c
                bac_match = str2
                print("No. of matching characters are: ", best_affinity);
        return best_affinity, str(bac_match)



'''arena = 100
bac = Chemotaxis(100,10)  # Object for chemokines is created
chemo = bac.chemo_attractants([1, 0, 1, 0, 1, 1, 0, 1, 1, 1], [7, 11, 12, 17, 11, 12, 19, 17, 13, 13],[81, 81, 73, 80, 76, 81, 80, 79, 79, 79])

# Object for monocytes is created

monocytes = Nk_cells(100, 10)  # Object for monocytes is created
mono_x,mono_y = monocytes.placement()  # Monocytes are placed
mono_loca = monocytes.arrayupdation()  # Array of monocytes is updated

# Object for monokines is created

a = Chemotaxis(100, 10)
monokines = a.chemo_attractants([1] * 10, mono_x, mono_y)

gradientx = monocytes.gradient(chemo, monokines)
plt.title('Monocytes Movement ')
plt.xlabel('Monocytes-X')
plt.ylabel('Monocytes-Y')
plt.imshow(mono_loca + monokines + chemo)
plt.show()'''
