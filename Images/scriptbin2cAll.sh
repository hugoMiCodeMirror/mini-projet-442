#!/bin/bash

if [ ! -d "image_h" ]; then
	mkdir image_h
fi

# Chemin vers le dossier avec les images .bmp
bmp_folder="../images_bmp"

for file in "$bmp_folder"/*.bmp; do
	if [ -f "$file" ]; then
		output_file="image_h/${file%.bmp}.h"
		./bin2c "$file" "$output_file"
	fi
done
