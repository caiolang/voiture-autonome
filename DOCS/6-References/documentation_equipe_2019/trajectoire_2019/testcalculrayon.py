import matplotlib.pyplot as plt
import numpy as np

import copy
from zonesafe import *
from adaptevitesserelat import *
from trouvecible import *
from calculrayoncourbure import*

orientation=0
orientationm1=0
N=720
lidar1=[]
lidar2=[]
rv=20
m=5
i=0
r1=50
r2=45
epsilon=0.15
alpha=60 #angle cone correction
v=100
deltat=0.1
rmax=1000

while i<N:
    lidar1.append((i,r1))
    i+=1
i=0
while i<N:
    lidar2.append((i,r2))
    i+=1

MMMR=zonesafe(lidar2,rv,m)
MMMRC=adaptevitesserelat (lidar1,lidar2,MMMR,alpha,v,deltat,rmax,orientation,orientationm1)
cible=trouvecible(MMMRC)
R=calculrayoncourbure(cible)

    
i=0
while i<len(MMMR):
    plt.plot(MMMR[i][2], MMMR[i][3],"b:o")
    
    plt.plot(MMMR[i][4], MMMR[i][5],"r:o")
    plt.plot(MMMRC[i][4], MMMRC[i][5],"g:o")
    
    i=i+1
i=0
while i<len(MMMR):
    plt.plot(r1*cos(i*360/N), r1*sin(i*360/N),"c:o")
    i+=1

#dessein de l'arc
thetamax=atan(cible[0]/R-cible[1])
thetainc=thetamax/N
i=0
arc=[]
while i<N:
    arc.append([R*sin(i*thetainc),R*(cos(i*thetainc)-1)])
    plt.plot(arc[i][0],arc[i][1],"k:o")
    i+=1

plt.plot(cible[0],cible[1],"y:o")
plt.axis('equal')
plt.show()




#calculerayoncourbure testé avec succés