**Objectifs de la séance :**
* Réunion avec Éric Fenaux pour discuter de la simulation
* Calculer le couple nécessaire au servomoteur
* Choisir le servomoteur
* Choisir l'accéléromètre
* Choisir le compte-tour
* Continuer le codage des programmes pour le lidar
* Continuer le site internet

D'après Éric Fenaux, la géométrie de la voiture fait qu'on n'a pas besoin de beaucoup de couple pour le servomoteur.

**Pour la prochaine fois :**
* Se lancer dans la CAO de la voiture
* Souder les batteries
* S'occuper de la carte de puissance
* Titouan : faire tourner la simulation sur la Raspi et vérifier qu'il n'y a pas de pbl
* Madleeine : voir comment on peut transférer le code de Titouan en C

# Réunion avec Éric Fenaux

“rejoindre la ligne droite le plus vite possible” : que veut dire le plus vite possible ? 
Asservissement de l’angle (?)
Aller regarder la recherche sur le tir de missile (à la bibli, il y a sûrement eu des cours là dessus à l’ENSTA) : torpille par exemple


Time to collision : pour la prise de décision
Est-ce qu’on peut décélérer pour percuter la cible avec une vitesse inférieure à ?? OU Est-ce qu’on a le temps de s’arrêter avant la collision ?


Quels sont les choix qui vous ont amenés à la stratégie pour rejoindre la ligne souhaitée ?\
Présenté la dernière fois : je veux rejoindre la courbe en 1s / 0,8s.


Processus implémenté sur le 2e programme : regarde les différents rayons de courbure que l’on peut adopter selon notre vitesse (accélération latérale maximale)\
Si le point voulu n’est pas atteignable avec un rayon de courbure :\
*1er temps :* calculer la nouvelle vitesse et le nouveau rayon de courbure pour atteindre le point voulu\
*2e temps :* manœuvre d’évitement : freiner et braquer à fond pour rendre le point atteignable


Stratégie de suivre les autres voitures en pensant que leur stratégie est bonne est mauvaise : on risque surtout de faire les même erreurs que le conducteur de devant.\
Notre algorithme permet de se battre contre des voitures très performantes, mais pour notre compétition il faut une stratégie plus prudente mais plus robuste.\
Si on supprime le module de suivi, est-ce qu’au global on gagne ou on perd du temps ? On en gagne à certains endroits mais on en perd quand ça mène à une erreur.


**Pour une présentation :** la faire au préalable devant un public candide ET qu’ils soient capables de réexpliquer les choses à la fin.


Si on fait tourner que la voiture noire sans la voiture rouge, est-ce qu’on est répétable en temps de tour ?
Pas testé sur cet algo, mais sur un algo précédent c’était le cas.


Les difficultés pour cet algo sont les angles droits, donc Titouan n’a pas fait plus de courbes pour se mettre dans la situation la plus difficile. Plus la surface est lisse, plus le lidar la mesure correctement, donc l’algo peut prendre les bonnes décision. Le problème des angles aigus est que par discrétisation de la détection de l’environnement par le lidar, le point de l’angle peut ne pas être détecté.


**Modèle de la voiture :** accélération longitudinale, accélération latérale maximale. Accélération longitudinale maximale (moteur) et freinage maximal (adhérence des pneus).\
Dans un premier temps, pas besoin d’angle volant, uniquement le ax et ay. Peut-on avoir simultanément ax et ay max ?
Non : on ne peut pas, à cause de l’effort de cisaillement sur le caoutchouc du pneu, qui est égal à la somme des deux contraintes. Entre le glissement longitudinal du pneu, il y a un domaine où la force longitudinale est égale au glissement du pneu (??? probablement mal noté).\
Si on est au sommet du domaine où le comportement est linéaire dans les deux directions, ça part. Si toute l’adhérence est mobilisée en longitudinal, il n’y a plus rien en latéral. Le axmax est celui de mon domaine linéaire, et à partir de là il faut définir une ellipse.\
Évitement où on freine à fond et braque à fond => glissement. Il faut utiliser l’ellipse pour faire une commande qui évite le patinage de la voiture.\
Pour l’instant, l’accélération a été sous-estimée par la voiture pour éviter le patinage.


**Pour rendre le truc inattaquable :** 
 - voici les hypothèses que j’ai faites
 - on pourrait faire ça pour rendre le truc mieux\
Permet d’expliquer les différences avec la vraie voiture.


**Capteurs :** 
 - lidar
 - compte-tour sur une roue arrière pour avoir la vitesse longitudinale
 - accéléromètre : pour l’accélération transversale, qu’on doit comparer avec l’accélération transversale voulue


Se localiser dans une carte du circuit avec le lidar : il faut des points particuliers repérables. Le lidar est pauvre en points particuliers, surtout si le circuit est lisse. Il ne faut que se recaler une fois par tour. À combiner avec l’algo actuel. Si on se perd, la prédiction rend la main à l’algo actuel.