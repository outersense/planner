from PIL import Image

# Define the size of the image (width and height)
n = 727   # Replace with your desired width
m = 250  # Replace with your desired height

# Create a new white image with the specified size
white_image = Image.new('RGB', (n, m), color='white')

# Save the white image to a file (optional)
white_image.save('white_image.png')  # You can change the file format as needed

# Show the white image (optional)
white_image.show()
