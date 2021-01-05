from math import *

def calculrayoncourbure(cible):
    "calcule le rayon de courbure necessaire à rejoidre cible depuis zero en ne prenant comme direction de v la direction dans laquelle pointe le véhicule"
    
    xp=cible[0]
    yp=cible[1]
    if yp!=0:
        K=-xp/yp #la pente de la droite ortho à O cible"
        C=yp/2-K*xp/2
    
        yo=K*0+C #intersection entre la droite portée par le rayon issu de O et la mediatrice de OCible se coupent en O et la droite portée par le rayon issu de O est d'équation x=0"
    if yp==0:
        yo=10000
    R=yo
    return(R)