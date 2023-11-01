import cv2
import numpy as np
import matplotlib.pyplot as plt

class LaneImageAnalyzer:
    def __init__(self, image_path):
        self.image_path = image_path
        self.image = cv2.imread(image_path)

    def preprocess_image(self):
        # Convert the image to grayscale
        self.gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)

        # Apply Canny edge detection
        self.edges = cv2.Canny(self.gray, threshold1=50, threshold2=150)

    def detect_intersection_points(self):
        # Use Hough Line Transform to detect lines
        lines = cv2.HoughLines(self.edges, 1, np.pi / 180, threshold=100)

        # Initialize a list to store intersection points
        self.intersection_points = []

        # Find intersection points
        for line1 in range(len(lines)):
            for line2 in range(line1 + 1, len(lines)):
                rho1, theta1 = lines[line1][0]
                rho2, theta2 = lines[line2][0]

                A = np.array([[np.cos(theta1), np.sin(theta1)],
                              [np.cos(theta2), np.sin(theta2)]])
                b = np.array([rho1, rho2])
                intersection = np.linalg.solve(A, b)

                # Check if the intersection point is within the image boundaries
                if 0 <= intersection[0] < self.image.shape[1] and 0 <= intersection[1] < self.image.shape[0]:
                    self.intersection_points.append((int(intersection[0]), int(intersection[1])))

    def draw_intersection_circles(self):
        # Draw circles around the intersection points
        for point in self.intersection_points:
            cv2.circle(self.image, point, 5, (0, 0, 255), -1)  # Red circle

    def display_image(self):
        # Display the image using matplotlib
        plt.imshow(cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB))
        plt.show()

    def process_and_display(self):
        self.preprocess_image()
        self.detect_intersection_points()
        self.draw_intersection_circles()
        self.display_image()

# Example usage:
if __name__ == '__main__':
    analyzer = LaneImageAnalyzer("output_image.jpg")
    analyzer.process_and_display()
