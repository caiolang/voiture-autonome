**Objectifs de la réunion :** Cours donné par M Eric Fenaux sur la dynamique du véhicule

**Présents :** Tout le monde, plus quelques personnes supplémentaires

## Dynamique du véhicule

[slides du cours](uploads/2b95c03fcadb99680b139b66fc10a2d0/autonome_diapos.pdf) \
[slides annexes sur le conducteur](uploads/72782301fff4e905cde3e79924805b83/commande_conducteur.pdf)

Tout ce qu'on peut faire par simulation et en rassemblant les connaissances peut être beaucoup plus puissant que ce qu'on peut faire par essai-erreur.

**Dynamique du véhicule :** Étude de l'accélération dans toutes les directions.\
*Repère utilisé :* repère orthonormé direct avec x en avant et z en vertical. 
* Angle autour de x : roulis
* Angle autour de y : tangage (?)
* Angle autour de z : cap

Accélération selon z : comportement / trajectoire VS accélération selon x et y : confort.

**Le but de la dynamique du véhicule est d'assurer la meilleure synthèse entre confort et comportement** : suivre la trajectoire demandée en isolant les passagers des déformations de la route. \
C'est semblable à un asservissement: on veut que la voiture suive les consignes du conducteur en étant le moins sensible possible aux perturbations extérieures (nids-de-poule, vent etc). 

*Exemple avec l’amortisseur d’une voiture :*\
Pour assurer le confort des passagers, on essaie que la fréquence propre des amortisseurs soit proche de la fréquence de la marche. Il faut également minimiser l’accélération de la caisse et minimiser la variation de charge de la caisse. 

Conseil : donner des indications au conducteur pour savoir très tôt si il y a un problème avec les commandes et qu’il faut apporter des corrections. 

La vitesse de lacet est dirigée par un angle de volant. *cf fomules dans les slides*\
**L’angle de dérive** est l’angle entre l’axe du véhicule et la vitesse. À plus de 6°, conducteur et passagers sont paniqués. 

**Résistance au roulement** : Force nécessaire pour faire la déformation de la roue au sol. *cf schémas dans les slides*

La meilleure façon de savoir où on est est par rapport aux limites d’adhérence de la voiture est un capteur d’accélération latérale (les autres indicateurs sont la vitesse de lacet et la dérive).

**Surbraquage** : Écart de dérive entre les pneus avant et les pneus arrière. \
Une voiture est *sous-vireuse* si son surbraquage augmente avec l’accélération transversale (c’est le cas de toutes les voitures ajd). \
Une voiture est *sur-vireuse* si le surbraquage diminue avec l’accélération transversale. \
Une voiture est *neutre* si le surbraquage ne varie pas avec l’accélération transversale.

La répartition des efforts est imposée par la répartition des masses.

**Bilan** : 
* La dérive arrière et le surbraquage ne dépendent que de l’accélération transversale (=latérale)
* L’angle volant pour obtenir la dérive avant varie en fonction de la vitesse, et cette dépendance est exlpicitée par l’angle d’Ackermann.
* Angle de braquage = angle d’Ackermann + surbraquage
 = L/R + taux de surbraquage * accélération latérale \
 Or accélération latérale = V²/R, d’où angle de braquage = (1/R) * (L + taux de surbraquage * V²)

**Pour optimiser les angles de braquage sur le parcours :**
* Caractériser la voiture : accélération transversale maximale, temps d’accélération. \
Faire tourner la voiture en rond pour différents rayons en faisant varier la vitesse et voir quel est l’angle de volant à mettre pour maintenir la courbe. \
Il faut obtenir la courbe de surbraquage. **Le faire avant la course sur le revêtement qu’on utilisera !!**
* Réussir à commander le rayon et donc l’accélération latérale voulue : dans un premier temps en boucle ouverte en régime permanent, puis ajouter une boucle de correction. 
* Bien choisir sa trajectoire : plus le temps d’accélération est court, plus il faut tourner court. \
Si le plan est connu, choisir la trajectoire optimale. Sinon, choisir une trajectoire de type conducteur. \
Faire des simulations ! \
Dans un premier temps, découpler les commandes : d’abord on décélère, puis on tourne, puis on accélère.

**Pour doubler qqn**, il faut viser un point de trajectoire à côté, laissant la place de le doubler. Il faut prendre en compte sa vitesse relative et le temps nécessaire pour le rattraper (time to collision). \
À ce moment là, il faut oublier qu'on fait un chrono : le but devient de le doubler, et choisir sa trajectoire en fonction : tirer tout droit, freiner tard, aller à la corde dès qu'on peut. On le distancera plus tard. \
Cependant, cette question est un peu prématurée pour nous. Il faut déjà essayer de faire une voiture qui roule correctement...

Idée : Faire un tour de **reconnaissance** pour prendre les mesures du terrain

**Ne pas se fixer comme objectif de gagner ! Se fixer des objectifs absolus (ex : un temps au tour) qui ne dépendent pas de ce que font les autres concurrents (et éviter les déceptions...)**