# Projet_nintendo

## Objectif : 
Créer un jeu video sur STM32 inspiré des licenses Pokemon et Zelda

## Principe :  
Le joueur (LINK) se déplace sur une carte sur laquelle il peut croiser des pokemons et les combattre. L'objectif du joueur est de rassembler les fragments de la triforce pour vaincre Ganondorf.

## Controles :
- joystick pour se déplacer et déplacer le curseur lors des choix
- bp1 pour sélectionner et interragir
- bp2 pour ouvrir le menu

## Contenu  : 
- 5 cartes différentes (gestion des murs et interactions avec les objets)

![map1](https://github.com/Coline3003/Projet_nintendo/assets/116337158/d1932ed6-69c1-4684-bf36-43cd2a4e02e5)

- 8 Pokémons différents capturables + un boss
- 2 types d'objets : les potions et les fragments de la triforce
- Le joueur peut posséder jusqu'a 4 pokemons
- Combat contre les pokemons en tour par tour avec la possibilité de changer de pokemon à la place d'attaquer

![im2](https://github.com/Coline3003/Projet_nintendo/assets/116337158/472b7f26-7dc0-4d71-929a-2428a2375801)


- menu pour la gestion de la position des pokemons dans l'équipe et l'utilisation des potions (c'est le premier pokemon de l'équipe qui est automatiquement sélectionné lors du lancement d'un combat)

![im1](https://github.com/Coline3003/Projet_nintendo/assets/116337158/e21f79dc-cb6b-46bf-a59e-00f425e9530e)


- 4 types de pokemons (normal, feu, eau, plante)
- gestion des degats (prise en compte des statistiques du pokemon, de son type, du choix de l'attaque, de son niveau, des statistique du pokemon adverse, du type du pokemon adverse)
- gestion de l'expérience acquise lors des combat (monté de niveau des pokemons)
- 3 musiques (une pour la séquence de démarrage, une pour le déplacement sur la carte et une pour les combats)

## Ressource STM32 utilisées :

### Périphériques

- GPIO pour la lecture des valeurs des boutons et du joystick
- SAI pour la gestion du son, utilisé avec la bibiliothèque BSP_AUDIO
- Ecran via la bibliothèque BSP_LCD

### FreeRTOS 

Utilisation des tâches, des fils d'attentes, interruptions ...

![diagramme](https://github.com/Coline3003/Projet_nintendo/assets/116337158/e114918d-ee91-4907-933f-e5b9c512a4af)

### FATFS

- Lecture des images au format .BMP pour le fond
- Lecture des fichiers audio au format .WAV
