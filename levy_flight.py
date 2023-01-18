import numpy as np
import random
import math
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Qt5Agg') #use Qt5 as backend, comment this line for default backend


def rotation(p, thetas):
    simulation = [random.randrange(8) * 1 for _ in range(16)]  # Randomly choose between 0-7 for 16 times
    freq = [simulation.count(i) for i in range(8)]  # Count the occuring of each number(event)
    directions = np.array([float(i * j) for i, j in zip(p, freq)])  # Possibility of each direction
    suitable_theta = np.argmax(directions)  # Select the maximum likelihood direction
    #r_rot = False  # To get random straight movement after random rotation
    #print('Robot will move randomly at an angle of')
    return thetas[suitable_theta]


thetas = {0: 0.0, 1: 45, 2: 90, 3: 135, 4: 180, 5: -135, 6: -90, 7: -45}
p = [0.125] * 8
initial = np.random.randint(3, 10, 2)
brow_initial = np.random.randint(3, 10, 2)
x = [initial[0]]
y = [initial[1]]
brow_walk_x = [initial[0]]
brow_walk_y = [initial[1]]
for t in range(150):
    if t % 15 == 0:
        l = random.randint(10, 15)
        brow_walk = random.randint(1, 2)
    else:
        brow_walk = random.randint(1, 2)
        l = random.randint(1, 2)
    theta = rotation(p, thetas)
    next_p = np.rint(initial + (np.array([math.cos(theta * math.pi/180), math.sin(theta * math.pi/180)]) * l))
    brow_walk_next_p = np.rint(brow_initial + (np.array([math.cos(theta * math.pi/180), math.sin(theta * math.pi/180)]) * brow_walk))
    x.append(next_p[0])
    y.append(next_p[1])
    brow_walk_x.append(brow_walk_next_p[0])
    brow_walk_y.append(brow_walk_next_p[1])
    initial = next_p
    brow_initial = brow_walk_next_p

plt.title('Levy Flight VS Browninan Random Walk')
plt.xlabel("x-axis")
plt.ylabel("y-axis")
plt.legend
plt.plot(x, y, label='Levy Flight')
plt.plot(brow_walk_x, brow_walk_y, label='Brownian Random Walk')
plt.legend()
plt.show()






