from PIL import Image, ImageDraw

# Image size (adjust as needed)
divideby =8
image_width = 400 / divideby
image_height = 250 / divideby

# Create a new image with a black background
image = Image.new("RGB", (int(image_width), int(image_height)), "black")
draw = ImageDraw.Draw(image)

# Define the positions and sizes of the circles
circle_radius = 100 / divideby
circle_x1, circle_y1 = image_width // 4+20/ divideby, image_height // 2
circle_x2, circle_y2 = 3 * image_width // 4-20/ divideby, image_height // 2
inner_circle_radius = 40/ divideby

# Draw the two large circles filled with white
draw.ellipse(
    (circle_x1 - circle_radius, circle_y1 - circle_radius, circle_x1 + circle_radius, circle_y1 + circle_radius),
    fill="white",
    outline="white",
)
draw.ellipse(
    (circle_x2 - circle_radius, circle_y2 - circle_radius, circle_x2 + circle_radius, circle_y2 + circle_radius),
    fill="white",
    outline="white",
)

# Draw the two smaller concentric circles filled with black
draw.ellipse(
    (
        circle_x1 - inner_circle_radius,
        circle_y1 - inner_circle_radius,
        circle_x1 + inner_circle_radius,
        circle_y1 + inner_circle_radius,
    ),
    fill="black",
    outline="black",
)
draw.ellipse(
    (
        circle_x2 - inner_circle_radius,
        circle_y2 - inner_circle_radius,
        circle_x2 + inner_circle_radius,
        circle_y2 + inner_circle_radius,
    ),
    fill="black",
    outline="black",
)

# Save the image to a file
image.save("maps/figure8_track.png")

# Show the image (optional)
image.show()
