#!/bin/bash

if [ ! -d "images_h" ]; then
	mkdir images_h
fi

# Chemin vers le dossier avec les images .bmp
bmp_folder="images_bmp_color"

for file in "$bmp_folder"/*.bmp; do
	if [ -f "$file" ]; then
		output_file="images_h/$(basename "$file" .bmp).h"
		./bin2c "$file" "$output_file"

		echo "Image traitée: $(basename "$file")"
	fi
done
