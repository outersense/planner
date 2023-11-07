import numpy as np

# Load the .npy file
npy_file_path = 'src/waypoints_Nov6_2bag.npy'
data = np.load(npy_file_path)

# Multiply x and y by 10
# data *= 10
scale_factor = 10

# Define translation values
# -2.041996779977085             3.9681654685361023
x_translation = -2.041996779977085
y_translation = 3.9681654685361023

# Translate x and y values
data[:, 0] = data[:,0]*scale_factor + x_translation
data[:, 1] = data[:,1]*scale_factor + y_translation

# Define the path for the modified .txt file
txt_file_path = 'src/waypoints_Nov6_2bag.txt'

# Save the modified data to the .txt file
np.savetxt(txt_file_path, data)

print(f'{npy_file_path} has been modified and saved to {txt_file_path}.')
