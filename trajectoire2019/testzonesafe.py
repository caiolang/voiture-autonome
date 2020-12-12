
import matplotlib.pyplot as plt
import numpy as np

#MMMR=[(thetai,ri,xi,yi,xpi,ypi)]


import copy
from zonesafe import *
N=720
lidar=[]
rv=20
m=5
i=0
r=50

while i<N:
    lidar.append((i,r))
    i+=1

MMMR=zonesafe(lidar,rv,m)
i=0
while i<len(MMMR):
    plt.plot(MMMR[i][2], MMMR[i][3],"b:o")
    plt.plot(MMMR[i][4], MMMR[i][5],"r:o")
    i=i+1
    
plt.axis('equal')
plt.show()

#zonesafe testé avec succés