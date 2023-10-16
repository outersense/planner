import cv2
import os
import numpy as np

folder_path = "maps"

# List all files in the folder
file_list = os.listdir(folder_path)

# Loop through the files in the folder
for file_name in file_list:
    if file_name.lower().endswith((".jpg", ".jpeg", ".png", ".gif", ".bmp", ".tiff")):
        # Construct the full path to the image file
        image_path = os.path.join(folder_path, file_name)

        # Read the image using OpenCV
        image = cv2.imread(image_path)

        if image is not None:
            # Get the height and width of the image
            height, width = image.shape[:2]
            print(f"Image: {file_name}, Width: {width}, Height: {height}")