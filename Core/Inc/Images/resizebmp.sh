#!/bin/bash

# Chemin vers le dossier contenant les images BMP à redimensionner
input_folder="images_bmp"

# Chemin vers le dossier où seront placées les images redimensionnées
output_folder="images_bmp_resized"

# Créer le dossier de destination s'il n'existe pas
mkdir -p "$output_folder"

# Largeur et hauteur de la nouvelle taille souhaitée
new_width=32
new_height=32

# Parcourir toutes les images dans le dossier d'entrée
for img in "$input_folder"/*.bmp; do
    # Extraire le nom de fichier sans extension
    filename=$(basename -- "$img")
    filename_no_ext="${filename%.*}"

    # Redimensionner l'image avec ImageMagick
    convert "$img" -resize ${new_width}x${new_height} "$output_folder/$filename_no_ext".bmp

    # Afficher le nom du fichier traité
    echo "Image traitée : $filename"
done

echo "Toutes les images ont été redimensionnées avec succès."
