

from math import *
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation

x = [0,1,2]
y = [0,1,2]
yaw = [0.0, 45, 45]
i=0
alpha=(45+180)*2*pi/360


while i<len(x):
    x[i]=x[i]+sqrt(2)/2*(cos(yaw[i]*2*pi/360+alpha))
    y[i]=y[i]+sqrt(2)/2*(sin(yaw[i]*2*pi/360+alpha))
    i+=1

fig = plt.figure()
plt.axis('equal')
plt.grid()
ax = fig.add_subplot(111)
ax.set_xlim(-10, 10)
ax.set_ylim(-10, 10)

patch = patches.Rectangle((0, 0), 0, 0, fc='y')

def init():
    ax.add_patch(patch)
    return patch,

def animate(i):
    patch.set_width(1.5)
    patch.set_height(1.0)
    patch.set_xy([x[i], y[i]])
    patch.angle = yaw[i]
    return patch,

anim = animation.FuncAnimation(fig, animate,
                               init_func=init,
                               frames=len(x),
                               interval=500,
                               blit=True)

plt.show()