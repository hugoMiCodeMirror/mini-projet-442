#!/bin/bash

# Chemin vers le dossier contenant les images BMP avec fond blanc
input_folder="images_bmp_resized"

# Chemin vers le dossier où seront placées les images modifiées
output_folder="images_bmp_color"

# Créer le dossier de destination s'il n'existe pas
mkdir -p "$output_folder"

# Demander à l utilisateur d entrer la couleur de fond au format hex
read -p "Entrez la couleur de fond au format hexadécimal (par exemple, #FFFFFF pour blanc) : " background_color_hex

# Validation du format
if [[ ! "$background_color_hex" =~ ^#([A-Fa-f0-9]{6}|[A-Fa-f0-9]{3})$ ]]; then
    echo "Format de couleur incorrect. Assurez-vous d'entrer un code hexadécimal valide."
    exit 1
fi

# Convertir la couleur de fond hexadécimale en format RGB
background_color_rgb=$(convert xc:"$background_color_hex" -format "%[pixel:p{0,0}]" info:-)

# Parcourir toutes les images dans le dossier d'entrée
for img in "$input_folder"/*.bmp; do
    # Extraire le nom de fichier sans extension
    filename=$(basename -- "$img")
    filename_no_ext="${filename%.*}"

    # Appliquer la transformation avec ImageMagick
    convert "$img" -fill "$background_color_rgb" -opaque white \
		   -fuzz 30% -fill "$background_color_rgb" -opaque "#f0f0f0" \
		   "$output_folder/$filename_no_ext""_$background_color_hex.bmp"

    # Afficher le nom du fichier traité
    echo "Image traitée : $filename"
done

echo "Toutes les images ont été traitées avec succès."
