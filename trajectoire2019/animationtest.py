from trajectoire import *


import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation

P=1600 #1 tour =375
traj=trajectoire(P)
x=traj[0]
y=traj[1]
yaw=traj[2]
[positioninit,orientationinit,vinit,deltat,amaxlat,epsilonmax,amax,amin,tsb,l,larg,vmax,N,rv,m,alpha,lanti]=params()
alpha=pi+atan(larg/l)


i=0
while i<len(x):
    x[i]=x[i]+rv*(cos(yaw[i]*2*pi/360+alpha))
    y[i]=y[i]+rv*(sin(yaw[i]*2*pi/360+alpha))
    i+=1
    
fig = plt.figure()
plt.axis('equal')

#afficher le circuit

envir=environment()
n=len(envir)
i=0
while i<n:
    nn=len(envir[i])
    j=0
    while j<(nn-1):
        plt.plot([envir[i][j][0],envir[i][j+1][0]],[envir[i][j][1],envir[i][j+1][1]],"c-.")
        j+=1
    i+=1
    
ax = fig.add_subplot(111)
ax.set_xlim(-1,18)
ax.set_ylim(-11,1)

patch = patches.Rectangle((0, 0), 0, 0, fc='k')

def init():
    ax.add_patch(patch)
    return patch,

def animate(i):
    patch.set_width(0.4)
    patch.set_height(0.2)
    patch.set_xy([x[i], y[i]])
    patch.angle = yaw[i]
    return patch,

anim = animation.FuncAnimation(fig, animate,
                               init_func=init,
                               frames=len(x),
                               interval=40,
                               blit=True)
                               
anim.save('tourss.html', writer='ffmpeg', fps=25)
plt.show()





