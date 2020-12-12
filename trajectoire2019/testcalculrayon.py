from math import *

def calculrayoncourbure(cible):
    "calcule le rayon de courbure necessaire à rejoidre cible depuis zero en ne prenant comme direction de v la direction dans laquelle pointe le véhicule"
    
    xc=cible[0]
    yc=cible[1]
    if yc!=0:
        K=xc/yc #la pente de la droite ortho à O cible"
        C=yc/2-K*xc/2
    
        yo=K*0+C #intersection entre la droite portée par le rayon issu de O et la mediatrice de OCible se coupent en O et la droite portée par le rayon issu de O est d'équation x=0"
    if yc==0:
        yo=100
    R=yo
    return(R)