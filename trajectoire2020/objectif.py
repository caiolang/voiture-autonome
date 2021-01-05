from math import *


def objectif(cible,v,deltat):
    K=atan(cible[1]/cible[0])
    xo=v*deltat/sqrt(1+K**2)
    yo=K*xo
    return([xo,yo])