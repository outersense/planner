# Read the input from the text file
with open('mapOS1.txt', 'r') as file:
    lines = file.readlines()

# Parse the input data
# N = int(lines[0].strip())
x_size, y_size = map(int, lines[1].strip().split())
print(x_size, y_size)
collision_threshold = int(lines[3].strip())

# Find the positions of L1 and L2
lane1_start = lines.index('L1\n') + 1
l1startx, l1starty = map(int, lines[lane1_start].strip().split())
lane1_end = lines.index('L2\n') - 1
l1endx, l1endy = map(int, lines[lane1_end].strip().split())
lane2_start = lines.index('L2\n') + 1
l2startx, l2starty = map(int, lines[lane2_start].strip().split())
lane2_end = lines.index('M\n') - 1
l2endx, l2endy = map(int, lines[lane2_end].strip().split())

# Create the map with 'M' for lane boundaries, collision threshold outside lanes, and 1.0 inside lanes
map_data = [['1.0' if (l1starty <= y <= l1endy and l1startx<= x < l2startx) or
                     (l2starty <= y <= l2endy and l1endx <= x < l2endx) else str(collision_threshold)
             for x in range(x_size)] for y in range(y_size)]

# Write the updated map back to the text file
with open('mapOS1.txt', 'a') as file:
    file.write('\nM\n')
    for row in map_data:
        file.write(' '.join(row) + '\n')

# Update the collision threshold in the file
# lines[3] = str(collision_threshold) + '\n'

# Write the updated data back to the text file
# with open('mapOS1.txt', 'w') as file:
#     file.writelines(lines)