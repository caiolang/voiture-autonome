import matplotlib.pyplot as plt
import numpy as np
import copy
from zonesafe import *
from adaptevitesserelat import *
from trouvecible import *
from environment import *
from lidar import *

#afficher le circuit

envir=environment()
n=len(envir)
i=0
while i<n:
    nn=len(envir[i])
    j=0
    while j<(nn-1):
        plt.plot([envir[i][j][0],envir[i][j+1][0]],[envir[i][j][1],envir[i][j+1][1]],"c:o")
        j+=1
    i+=1
    
i=0
position=[11.5,-9.5]
orientation=0
N=360
rv=0.2
m=0.05
alphainc=360/N

M=lidar(envir,position,orientation,N)
MMMR=zonesafe(M,rv,m)
x=position[0]    
y=position[1]

while i<len(M):
    r=M[i][1]
    xl=r*cos((M[i][0]*alphainc+orientation)*2*pi/360)+x
    yl=r*sin((M[i][0]*alphainc+orientation)*2*pi/360)+y
    plt.plot(xl, yl,"g:o")
    i+=1
i=0
#callage de MMMR dans notre repère avec position et orientation
while i<len(MMMR):
    xl=MMMR[i][2] #abscisse du pt lidar dans le ref du lidar 
    yl=MMMR[i][3]
    theta=MMMR[i][0] #nb inc angle de coord polaires dans le ref du lidar
    thetac=(theta*alphainc+orientation)*2*pi/360  #angle de coord polaires dans le ref absolu
    r=MMMR[i][1]
    xlc=r*cos(thetac)+x #rcos(thetac) =abscisse dans le ref du lidar rotationné de orientation xlc=abscisse dans le ref absolu
    ylc=r*sin(thetac)+y

    xlp=MMMR[i][4] #abscisse du pt lidar dans le ref du lidar 
    ylp=MMMR[i][5]
    thetap=atan((ylp)/(xlp))
    if xlp<0:
        thetap=pi+atan(ylp/xlp)
    thetacp=thetap+orientation*2*pi/360
    rp=sqrt(xlp**2+ylp**2)
    xlpc=rp*cos(thetacp)+x
    ylpc=rp*sin(thetacp)+y


    plt.plot(xlc, ylc,"b:o")
    plt.plot(xlpc,ylpc,"r:o")
    i+=1
plt.axis('equal')   
plt.show()

#lidar testé avec succès après 6h de débugage
