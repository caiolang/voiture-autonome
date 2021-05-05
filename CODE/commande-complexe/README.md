# Voiture Autonome

DISCLAIMER : Il est très difficile de "rentrer" dans ce code adapté de celui de l'année 2019-2020. Ce guide ne vous évitera
pas de longues heures de prise en main, mais peut-être moins que nous (on espère).

## Instructions d'utilisation

Pour faire tourner la simulation, tapez

```
python SimVideo.py
(ou python3 SimVideo.py selon la distribution)
```

-----------------------------------

## Description des fichiers

### A) Acquisition de Données

#### 1. `Sensors.py`
Fonctions `getLidar()` et `turnServo()`, qui font l'interaction entre le LIDAR et Servo réels et la Raspberry Pi.

-----------------------------------

### B) Configuration

#### 1. `Parameters.py`
Les paramétres du programme: 
- Dimensions de la voiture
- Paramétres dynamiques de la voiture 
- Quelques paramétres auxiliaires utilisés dans le programme
- Paramétres utilisés pour simuler la voiture dans l'environnement simulé
- etc.

-----------------------------------

### C) Simulation

#### 1. `Simulation.py`
Fonctions utilisés pour calculer l'environment simulé a chaque pas de temps. La fonction Simulation_V2 correspond aux évolutions par rapport au code de base.
> Est appelé par `SimVideo.py`.

#### 2. `SimVideo.py`
Programme qui genére une simulation en vidéo de deux voitures dans un environnement, la voiture noire avec le code non modifié de 2019-2020 et la rouge avec les évolutions de 2020-2021.
> L'environnement et les fonctions utilisés sont définis dans `Simulation.py`.

-----------------------------------

### D) Math functions

Contient des fonctions de calcul d'intersection.
Appelées par 'RouteFunctions.py' et 'DataFunctions.py' (cf plus bas).
> Rien à signaler de particulier.

-----------------------------------

### E) Car functions

Contient "UpdateCar", qui met à jour les informations de la voiture telles que sa vitesse, son angle de braquage etc... (attention! le booléen "reverse" ne signifie pas marche arrière, il aurait plutôt du s'appeler "brake")
> Appelées par 'Simulation.py'

-----------------------------------

### F) Route functions

Contient les fonctions de recherche de cible et de trajectoire optimale. Pas d'évolutions majeures par rapport à 2019-2020.
> Appelées par 'Simulation.py'

-----------------------------------

### G) Data functions

Contient les fonctions de traitement des données simulées du Lidar.
La fonction Safezone_V2 est particulièrement cruciale et contient l'essentiel des évolutions par rapport à Safezone qui date de 2019-2020 (comparer les 2 et lire le rapport).
> Appelées par 'Simulation.py'
