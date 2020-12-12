À l'attention de Lucie : aide pour utiliser Markdown dans GtiLab https://gitlab.com/help/user/markdown#diagrams-and-flowcharts-using-mermaid et la suite de la page \
Ne pas mettre de / dans les noms des fichiers, ça crée un nouveau dossier
Mettre les dates à l'envers permet de classer automatiquement les compte-rendus par ordre chronologique.

Différents pôles :
========

- Mécanique : 
  - montage du châssis
  - montage carrosserie
  - gérer la connectique avec les moteurs, la carte, l’alimentation etc : 
  - choisir le servo-moteur pour gérer la direction 
  - gérer l’interaction PC/voiture (choix du hardware)
  - customisation de la voiture
  - gérer l’approvisionnement en pièces détachées

- Détection des obstacles et voitures et bordures
  - choix des capteurs (lidar pour bordure et caméra pour voitures et obstacles ?)
  - algorithme de transfert des données vers l’ordinateur
  - algorithmes d’exploitation des données

- Commande de la voiture, optimisation de la trajectoire
  - calcul de la trajectoire optimale à partir des données de détection : utiliser de l’IA ?
  - algo de transmission des commandes à la voiture
  - algorithme de commande des moteurs à partir de la commande de direction 

- CAO ? Sur un outil qui exporte en stl (à l’U2IS Inventor) Autodesk libre d’accès

**Attention aux interfaces !**

Premières étapes :
===

- Poser des questions organisationnelles aux profs référents : 
  - Quand est-ce qu’on commande ? 
  - Quel délai de livraison ? 
  - Est-ce qu’on commande nous-même ? Quels remboursements ? \
_Budget encore en discussion → sponsor ? Labo ?
Un sponsor est valorisé_
  - Est-ce qu’un langage de programmation est imposé ? \ 
_Non. C valorisé ? Documenter le choix_
  - Conseils sur la transmission avec l’ordinateur ? 
_Wifi, pas le choix\
Radio pourquoi pas? Carte de communication à acheter\
Calculs embarqués possibles ?_
  - Conseils pour de la littérature sur les voitures autonomes ? 
  - Est-ce qu’on doit participer à la course à la fin ? On ne sera pas là puisque c’est pendant les dates de stage\
_À voir, pour l’instant on ne sait pas\
Éventuellement un match amical avec d’autres écoles plus tôt dans l’année_


- Choix du hardware : vérifier le volume occupé : quelles combinaisons de capteurs et carte sont viables ? \_Quelle vitesse de calcul ? Quelle vitesse de transmission ?_
- Commande du hardware (voiture, capteurs, carte de commande : raspberry pi ?)
- Choix du langage de programmation
- Ensuite : commande du servomoteur de direction

Livrables : 
===

__Proposition d’outils à utiliser, pour faciliter la passation avec les élèves de l’année prochaine__
- Gestionnaire de version : gitlab (tutos sur openclassrooms), mettre tous les documents dessus (DRIVE INTERDIT !)
tuto data
- Page web de communication : photos, séances, compte-rendus, vidéos de test
- Middleware : ROS = outil qui a bcp de librairies infos et qui évite de faire du bas niveau (certains ont déjà programmé de l’évitement d’obstacle par ex)\
aller chercher des tutos 
- Plateforme de gestion de projet : trello (qui se lie avec slack) 
aller chercher des tutos
- CAO : sur un outil qui exporte en stl (à l’U2IS Inventor) Autodesk libre d’accès
- Page wiki (équivalent de README) où est détaillé où sont tous les documents
*But :* environnement clair pour tout le monde

**Attention aux interfaces ! Communiquer entre les pôles**

Désigner un chef de projet, un responsable qualité (potentiellement le chef de projet)\
*Responsable qualité :* vérifie que la documentation est claire (ISO 2021) (commenter les codes, git complet et bien tenu (peu de branches, commit correct, personne ne développe en local sur son ordi), toute la doc au même endroit, versionner les codes (faire des branches quand on veut faire des améliorations = cf tuto openclassrooms), etc)\
Un novice doit pouvoir l’utiliser 

### Désigner une organisation sur le suivi de projet :

*Réunions :* agenda (points qu’on veut aborder en réunion) et compte-rendu. \
Si pas de décision prise : mettre des actions en places, avec une deadline, et ajoutées sur trello.
- Réunions par pôles, réunions globales de groupe, réunions avec le tuteur (chef de projet et chef de pôle, une fois toutes les trois semaines) : demander des conseils sur des points précis. Réunions en faisant des points avec toute l’équipe et le tuteur (tous les mois et demi)
- Mettre en place l’organisation de travail

**Prise en main du sujet :**
- Définition du cahier des charges
- Décomposition exhaustive des étapes : faire un FAST, à envoyer avant la 2e séance
- Mise en place des jalons du projet

Pour la prochaine fois : 
====
- organisation par pôles définie + qqn pour la com, qqn pour la qualité, chef de projet
- Cahier des charges 
- FAST
- Ébauche du rétroplanning qui en découle

gabriel.ballet@ensta-paris.fr 
