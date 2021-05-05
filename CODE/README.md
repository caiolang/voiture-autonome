# Voiture Autonome

## Instructions de tournage

Pour tourner la simulation, tapez

```
python SimVideo.py
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
- Paramétres utilisés pour simuler la voiture dans l'environment simulé
- etc.

-----------------------------------

### C) Simulation

#### 1. `Simulation.py`
Fonctions utilisés pour calculer l'environment simulé a chaque pas de temps. Est utilisé par `SimVideo.py`.

#### 2. `SimVideo.py`
Programme qui genére une simulation de tournage de deux voitures dans un environment.
L'environment et les fonctions utilisés sont définis en `Simulation.py`.