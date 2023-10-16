import cv2
import numpy as np

# Image size (adjust as needed)
divideby = 4
image_width = int(400 / divideby)
image_height = int(250 / divideby)

# Create a new image with a black background
image = np.zeros((image_height, image_width, 3), dtype=np.uint8)

# Define the positions and sizes of the circles
circle_radius = int(100 / divideby)
circle_x1, circle_y1 = int(image_width // 4 + 20 / divideby), int(image_height // 2)
circle_x2, circle_y2 = int(3 * image_width // 4 - 20 / divideby), int(image_height // 2)
inner_circle_radius = int(40 / divideby)

# Draw the two large circles filled with white

for i in range(int((circle_radius-inner_circle_radius)/2)):
    # cv2.circle(image, (circle_x1, circle_y1), circle_radius-i, color, thickness) 
    color_ = int(0+i*255*2/(circle_radius-inner_circle_radius))
    cv2.ellipse(image, (circle_x1, circle_y1), (circle_radius-i, circle_radius-i), 0, 0, 360, (color_, color_, color_), 2)
    cv2.ellipse(image, (circle_x1, circle_y1), (inner_circle_radius+i, inner_circle_radius+i), 0, 0, 360, (color_, color_, color_), 2)
    cv2.ellipse(image, (circle_x2, circle_y2), (circle_radius-i, circle_radius-i), 0, 0, 360, (color_, color_, color_), 2)
    cv2.ellipse(image, (circle_x2, circle_y2), (inner_circle_radius+i, inner_circle_radius+i), 0, 0, 360, (color_, color_, color_), 2)

# cv2.ellipse(image, (circle_x1, circle_y1), (circle_radius, circle_radius), 0, 0, 360, (255, 255, 255), -1)
# cv2.ellipse(image, (circle_x2, circle_y2), (circle_radius, circle_radius), 0, 0, 360, (255, 255, 255), -1)

# # Draw the two smaller concentric circles filled with black
# cv2.ellipse(image, (circle_x1, circle_y1), (inner_circle_radius, inner_circle_radius), 0, 0, 360, (0, 0, 0), -1)
# cv2.ellipse(image, (circle_x2, circle_y2), (inner_circle_radius, inner_circle_radius), 0, 0, 360, (0, 0, 0), -1)

# Save the image to a file
cv2.imwrite("maps/figure8_track2.png", image)

# Show the image (optional)
cv2.imshow("Figure-8 Track", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
