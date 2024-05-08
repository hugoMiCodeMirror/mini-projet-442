# Snake Game

## Objectif
Créer un jeu classique de serpent (snake) avec une progression de la vitesse du serpent au fil du temps.

## Principe
Le jeu consiste à contrôler un serpent qui se déplace sur un tableau de jeu. Le serpent doit manger des pommes pour grandir. Cependant, si le serpent se heurte à lui-même ou aux bords de l'écran, la partie est terminée.

## Contrôles
- Utilisation du joystick pour déplacer le serpent.
- Utilisation de l'écran tactile pour redémarrer le jeu après la fin de la partie et mettre en pause la partie en cours.

## Ressources STM32 Utilisées
### Périphériques
- GPIO pour la lecture des valeurs du joystick.
- SAI (Serial Audio Interface) pour la gestion du son, utilisé avec la bibliothèque BSP_AUDIO.
- Écran LCD via la bibliothèque BSP_LCD.

### FreeRTOS
Utilisation des fonctionnalités de FreeRTOS :
- Tâches pour organiser les différentes parties du code.
- Files d'attente pour le réveil des tâches.
- Interruptions pour gérer les événements externes.
- Mutex pour la gestion des ressources partagées.

### FATFS
Lecture des fichiers audio au format .WAV grâce à la bibliothèque FATFS.

## Utilisation
1. Mettez les fichiers audio sur la carte SD présente dans le dossier Ressources du projet.
2. Insérez la carte SD dans le lecteur de la carte STM32.
3. Branchez un casque pour profiter de l'expérience sonore.
4. Utilisez le joystick pour déplacer le serpent et collecter des pommes.
5. Utilisez l'écran tactile pour redémarrer le jeu ou mettre en pause la partie en cours.