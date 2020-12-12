# LIDAR

Dans ce dossier il y a deux fichiers python.
Il faut brancher le LIDAR à un port USB de l'ordinateur ou de la carte Raspberry Pi pour les faire fonctionner.

## lidar_get.py

Ce programme nécessite l'installation du module rplidar disponible à l'adresse https://github.com/SkoltechRobotics/rplidar . Ce git possède une notice qui explique comment l'installer. Il y a aussi des exemples d'utilisation. 

Le programme renvoit à chaque tour complet du Lidar une liste d'élément de la forme [theta,r], avec theta l'angle Lidar et r la distance de l'obstacle capté pour cette angle.
Le programme sous cette forme est calibré pour ne se concentrer que sur les 180° de la vision avant du LIDAR puisque c'est tout ce qui est nécessaire à la simulation. Mais cela peut être modifié.
Attention, si le LIDAR ne capte pas d'obstacle pour un angle, cet angle ne sera pas pris en compte dans la liste de données. La taille de cette liste est donc susceptible de varier. 

La vitesse de rotation du LIDAR peut être définie dans le programme (lidar.set_pwm(votre-vitesse)). Attention la vitesse du LIDAR influe sur la précision et la fréquence d'acquisition. Il faut donc choisir la meilleure en prenant en compte les capacités de l'algorithme de calcul de trajectoire.

Ce programme n'est pas à exécuter de la sorte. Il faut l'importer dans le programme principal (import lidar_get). Il permet au programme principal d'acquérir les données LIDAR en temps réel.
Pour l'utiliser il suffit de faire une boucle qui parcourt lidar_get.run(). Ainsi à chaque nouveau tour de boucle un nouveau tableau de [theta,r] est disponible.

## lidar_test.py

Il s'agit d'un exemple d'utilisation pour comprendre le fonctionnement plus facilement.