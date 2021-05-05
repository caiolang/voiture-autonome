**Objectifs de la réunion :** Travail par pôle.\
On a enfin reçu le matériel ! On a fait une petite vidéo en ouvrant le colis, pour pouvoir la poster sur le site internet.\
 Une partie de l'équipe monte la voiture, Titouan commente les différents programmes de la simulation. On avance sur la gestion de configuration, notamment un nouveau GANTT et un rétroplanning.\
Séverine Bournaud est passée nous voir pour savoir où on en est dans le projet.


**Présents :** Tous sauf Gabriel et Lucie


## Mise à jour du planning prévisonnel

Comme nous l'avons dit à Séverine Bournaud, nous avons mis beaucoup plus de temps à obtenir le matériel que ce que nous avions envisagé. Pendant ce laps de temps nous avons avancé sur la gestion de configuration (notamment le FAST, cf les versions successives) et les simulations de la voiture. Le cours de M. Eric Fenaux a été très utile pour ce dernier point. Cependant, sans le matériel sous la main, il est difficile d'avancer. Il est notamment très difficile de dessiner les interfaces entre les moteurs, le lidar, la carte de commande (Raspberry Pi 4B) et le châssis. Le but de ce compte-rendu est de faire un bilan de ce qui reste à faire, d'adapter nos objectifs aux ressources (notamment en temps) que nous avons aujourd'hui, et d'essayer au maximum de respecter ces nouveaux objectifs.


**Les nouveaux objectifs de notre groupe :**\
Au début du projet, nous avions comme objectifs de faire une voiture autonome qui puisse participer à la course. Nous avions notamment voulu détecter les obstacles et les voitures adverses plus efficacement qu'avec seulement le lidar. En effet, puisque les voitures adverses sont de la même taille que notre propre voiture et que les obstacles sont probablement un peu plus petits, ils ne sont pas forcément détectés par le lidar placé sur le toit de la voiture. Nous voulions donc lui adjoindre une caméra, et l'utiliser pour faire de la reconnaissance des obstacles, probablement avec des réseaux de neurones. C'est pour cela que nous avions envisagé d'acheter une carte de commande Nvidia : elle possède une carte graphique dont la structure matérielle est très bien adaptée aux calculs en réseaux de neurones. Cependant, par manque de temps, nous ne pourrons pas nous consacrer à la caméra et à l'exploitation de ses images. Nous abandonnons donc cet objectif (cela avait déjà été mentionné dans un précédent compte-rendu).\
Le nouvel objectif est déjà de faire une voiture qui roule, et de faire fonctionner le lidar. Dans l'idéal, nous voulons réussir à utiliser le lidar pour faire rouler la voiture en autonomie.

**Objectifs principaux :** 
* Réussir à faire rouler la voiture avec un lidar fonctionnel
* Monter le site internet du projet
* Faire une documentation claire tout au long du projet, et pouvoir faire une bonne passation


**Objectif suivant (si on a du temps) :** Rendre la voiture autonome


Le but de ce paragraphe est de faire un bilan de tout ce qui nous reste à faire pour atteindre nos objectifs principaux, pour pouvoir ensuite faire un planning prévisionnel.\
Nous avons aujourd'hui reçu le châssis de la voiture. Cependant, nous nous sommes rendus compte que l'on a oublié de commander une batterie pour l'alimenter. Il faudra voir avec le laboratoire de robotique si ils ont du matériel à nous prêter pour faire une solution provisoire, mais je pense qu'il faudra de toute façon commander une batterie.\
Il nous faut aussi commander un servo-moteur pour pouvoir commander la direction. Sans le châssis, nous ne pouvions pas mesurer l'espace dont nous disposons dans la voiture pour l'installer, ni comment on peut le fixer à la fois au châssis et au train avant. Une fois que la voiture sera montée on pourra faire ces mesures. Il faudra probablement dessiner et imprimer des pièces 3D pour pouvoir fixer le servomoteur à la voiture.\
Il faut également se préoccuper de l'interface entre la carte de commande et les moteurs. La Raspberry Pi n'est pas une carte de puissance, on ne peut donc pas brancher les moteurs directement dessus. Il faudra commander ou faire une carte de puissance pour pouvoir commander les moteurs. Dessus seront proablement branchés la carte de commande, le lidar, le servomoteur et les moteurs des roues. En bref, tout ce qui nécessite d'être alimenté en électricité.\
En parallèle, il faut terminer de configurer la carte Raspberry Pi et écrire les programmes de commande des moteurs et de réception des données du lidar.\
Quand la partie mécanique et électronique sera terminée, nous pourrons faire des mesures sur la dynamique de notre voiture (cf cours de M. Fenaux), pour pouvoir améliorer notre commande de la voiture.

Il ne faut pas non plus oublier de mettre à jour le site internet, ni de poursuivre la gestion de configuration de façon exhaustive au fur et à mesure de nos avancées.


**Tâches à réaliser :**
* Monter le châssis
* Trouver/commander des batteries
* Mesurer la place disponible pour le servomoteur et les fixations possibles
* Choisir et commander le servomoteur
* Si besoin, dessiner et imprimer des pièces de fixation pour le servomoteur
* Étudier l'interface de puissance
* Choisir ou réaliser la/les carte(s) de puissance
* Écrire les programmes de commande des moteurs
* Étudier l'interface entre le lidar et la carte de commande
* Écrire le programme qui récupère les données du lidar
* Faire les mesures de dynamique de la voiture
* Écrire un programme permettant de faire suivre une trajectoire précise et pré-déterminée à la voiture
* Terminer de déterminer l'hébergement de notre site avec data
* Vérifier l'installation de Wordpress
* Développer les pages d'après la structure du site décidée (cf home/Site)


Suite à cette liste de tâches, Madeleine va réaliser une nouvelle version du GANTT.