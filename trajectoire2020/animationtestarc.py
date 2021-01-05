from trajectoirearc import *


import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation

P=500 #1 tour =?
[traj1,traj2]=trajectoirearc(P)

x1=traj1[0]
y1=traj1[1]
yaw1=traj1[2]

x2=traj2[0]
y2=traj2[1]
yaw2=traj2[2]

[positioninit1,positioninit2,orientationinit1,orientationinit2,vinit1,vinit2,deltat,amaxlat,epsilonmax,amax,amin,tsb,l,larg,vmax,N,rv,m,alpha,lanti]=params()
alpha=pi+atan(larg/l)


i=0
while i<len(x1):
    x1[i]=x1[i]+rv*(cos(yaw1[i]*2*pi/360+alpha)) #coordonnées du coin inférieur gauche
    y1[i]=y1[i]+rv*(sin(yaw1[i]*2*pi/360+alpha))
    x2[i]=x2[i]+rv*(cos(yaw2[i]*2*pi/360+alpha)) #coordonnées du coin inférieur gauche
    y2[i]=y2[i]+rv*(sin(yaw2[i]*2*pi/360+alpha))
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

patch1 = patches.Rectangle((0, 0), 0, 0, fc='k')
patch2 = patches.Rectangle((0, 0), 0, 0, fc='r')


def init():
    ax.add_patch(patch1)
    ax.add_patch(patch2)
    return (patch1,patch2)

def animate(i):
    patch1.set_width(0.4)
    patch1.set_height(0.2)
    patch1.set_xy([x1[i], y1[i]])
    patch1.angle = yaw1[i]
    patch2.set_width(0.4)
    patch2.set_height(0.2)
    patch2.set_xy([x2[i], y2[i]])
    patch2.angle = yaw2[i]
    return (patch1,patch2)

anim = animation.FuncAnimation(fig, animate,
                               init_func=init,
                               frames=len(x1),
                               interval=100,
                               blit=True)

#anim.save('tours.html', writer='ffmpeg', fps=10)
plt.show()



