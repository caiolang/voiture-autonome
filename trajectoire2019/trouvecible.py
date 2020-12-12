from math import *
from intersection import *
import matplotlib.pyplot as plt

def trouvecible (MMMRC):
    "MMMRC=[(thetai,ri,xi,yi,xpi ou xpprimi,ypi ou ypprimi)]"
    "MMMRCT=[(thetai,ri,xi,yi,xpi ou xpprimi,ypi ou ypprimi,rpi)] trié par rpi croissant"
    "segments=[(xpprimi,ypprimi,xpprimip1,ypprimip1)]"
    
    MMMRCT=[]
    NNN=len(MMMRC)
    i=0 
    while i<NNN:
        d=MMMRC[i][4]**2+MMMRC[i][5]**2
        liste=MMMRC[i]+[d]
        MMMRCT.append(liste)
        i+=1
        
    MMMRCT.sort(key=lambda x: x[6])
    MMMRCT.reverse()
    segments=[]
    i=0
    while i<(NNN-1):
        segments.append([MMMRC[i][4],MMMRC[i][5],MMMRC[i+1][4],MMMRC[i+1][5]])
        
        #plt.plot([MMMRC[i][4]+4,MMMRC[i+1][4]+4],[MMMRC[i][5]-1,MMMRC[i+1][5]-1],"y-+") #debugage
        i=i+1
        
    i=0
    while i<NNN:
        xpi=MMMRCT[i][4]
        ypi=MMMRCT[i][5]
        j=0
        while j<len(segments) and intersection((xpi,ypi),segments[j])!=1:
            j+=1
        if j==len(segments): #alors le pt pi trouvé une cible qui ne coupe aucun segment"
            return([xpi,ypi])
        i+=1
    print('ERREUR pas de cible optimale')
    return('ERREUR')
    #return([MMMRCT[0][4],MMMRCT[0][5]]) #debugage
    