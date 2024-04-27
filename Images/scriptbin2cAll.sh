#!/bin/bash

if [ ! -d "image_h" ]; then
	mkdir image_h
fi


for file in *.bmp; do
	if [ -f "$file" ]; then
		output_file="image_h/${file%.bmp}.h"
		./bin2c "$file" "$output_file"
	fi
done
