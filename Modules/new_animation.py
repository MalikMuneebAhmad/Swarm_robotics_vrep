import matplotlib
matplotlib.use('Qt5Agg') #use Qt5 as backend, comment this line for default backend
from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np
import random


fig = plt.figure()
ax = plt.axes(xlim=(0, 100), ylim=(0, 100))
N = 1
images = [plt.imshow(np.random.rand(random.randint(1,100),random.randint(1,100))) for _ in range(N)] #lines to animate
def init():
    for img in images:
        img.set_data(np.zeros([1000,1000]))
    return images #return everything that must be updated
def animate(i):
    for img in images:
        img.set_data(np.random.rand(random.randint(1,100),random.randint(1,100)))
        #img.set_data(np.zeros ([1000,1000]))
    return images
anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=100, interval=20, blit=True)

plt.show()
